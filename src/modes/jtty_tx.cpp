// src/modes/jtty_tx.cpp — JTTY-передатчик (WSJT-X 3.3 совместимый).  
//  
// Кодек (pack28, ihashcall, атомы, CRC12 + tail-biting CC K=10) — порт  
// из jtty_modem.cpp проекта RP2040_beacon, который в свою очередь портирован  
// из packjt77.f90 / jtty_source_codec.f90 / tbcc.f90 проекта WSJT-X  
// (Joe Taylor K1JT, Steve Franke K9AN и команда WSJT, GPLv3).  
//  
// Модулятор — по модели rtty_tx.cpp: один NCO (LUT-синус), 4 шага фазы,  
// на границе символа переключается только шаг — CPFSK без щелчков.  
// Длительность символа 32 мс при AF_FS_OUT_HZ=15464 = 494.848 отсчёта —  
// не целое, поэтому длина каждого символа считается накопителем Q16  
// (чередование 494/495), средняя скорость ровно 31.25 Бод без дрейфа.  
//  
// Разделение по ядрам (как rttytx):  
//   Core 0: jttytx_init/send/poll — упаковка и символьный автомат.  
//   Core 1: jttytx_sample() — NCO + выбор шага, только int32/LUT.  
// ============================================================================  
#include "jtty_tx.h"  
#include "../af_config.h"     // AF_FS_OUT_HZ  
#include "af_oled.h"          // эхо передаваемого текста на OLED  
#include <math.h>  
#include <string.h>  
#include <ctype.h>  
  
// ---------- Константы протокола ----------  
#define JTTY_BAUD_MILLIHZ   31250UL            // 31.25 Гц в мГц  
#define JTTY_NTONES         4  
#define JTTY_NSYNC          13  
#define JTTY_NDATA          46  
#define JTTY_NSYMS          59                 // 13 + 46  
#define JTTY_MAX_FRAMES     16  
// f0 = центр - 1.5*baud = 1000 - 46.875 = 953.125 Гц  
#define JTTY_F0_MILLIHZ     (JTTYTX_CENTER_HZ * 1000UL - JTTY_BAUD_MILLIHZ * 3UL / 2UL)  
  
static const uint8_t JTTY_IS13[JTTY_NSYNC] =  
    {0,2,2,3,0,0,3,2,1,3,1,2,0};  
  
// Алфавит источника (jtty_source_codec.f90), 64 символа, индексы 0..63  
static const char JTTY_ALPHABET[] =  
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ +-./?!\"#$%,&*()_'=[]{}<>|:;";  
  
// ---------- Типы кодека ----------  
typedef struct {  
    int kind;       // JTTY_ATOM_*  
    int subtype;  
    int role;       // JTTY_ROLE_*  
    int value;  
    int value2;  
    char text[14];  
} JttyAtom;  
  
typedef struct {  
    uint64_t frames[JTTY_MAX_FRAMES];   // 34-битные payload'ы  
    int nframes;  
} JttyFrameList;  
  
enum {  
    JTTY_ATOM_CALL = 0, JTTY_ATOM_EXCH_NUM = 1, JTTY_ATOM_EXCH_LOC = 2,  
    JTTY_ATOM_EXCH_PAIR = 3, JTTY_ATOM_EXCH_NUM_TIME = 4,  
    JTTY_ATOM_CONTROL = 5, JTTY_ATOM_GRID4 = 6, JTTY_ATOM_TEXT5 = 7  
};  
enum { JTTY_CALL_CQ=0, JTTY_CALL_CALL=1, JTTY_CALL_TU_CQ=2,  
       JTTY_CALL_CALL_TU=3, JTTY_CALL_CALL_AGN=4, JTTY_CALL_TU_NOW=5 };  
enum { JTTY_ROLE_FIELD_ONLY=0, JTTY_ROLE_FULL=1 };  
enum { JTTY_EXCHANGE_UNKNOWN=0 };  
  
enum { JTTY_CONTROL_AGN=0, JTTY_CONTROL_CALL=1, JTTY_CONTROL_AGN_CALL=2,  
       JTTY_CONTROL_NR=3,  JTTY_CONTROL_AGN_NR=4, JTTY_CONTROL_EXCH=5,  
       JTTY_CONTROL_STATE=6, JTTY_CONTROL_SECTION=7, JTTY_CONTROL_ZONE=8,  
       JTTY_CONTROL_GRID=9,  JTTY_CONTROL_RPRT=10, JTTY_CONTROL_QSL_TU=11,  
       JTTY_CONTROL_TU=12,   JTTY_CONTROL_QRZ=13, JTTY_CONTROL_QSO_B4=14,  
       JTTY_CONTROL_WAIT=15, JTTY_CONTROL_NIL=16,  JTTY_CONTROL_OK=17 };  
  
// ============================================================  
// Алфавит / утилиты  
// ============================================================  
static int source_index(char c) {  
    const char* p = strchr(JTTY_ALPHABET, c);  
    return p ? (int)(p - JTTY_ALPHABET) : -1;  
}  
  
// ============================================================  
// nhash — Jenkins hashlittle (эталон nhash.c), для токенов CQ_aaaa  
// ============================================================  
static inline uint32_t rot32(uint32_t x, int k) { return (x << k) | (x >> (32 - k)); }  
  
static inline void nh_mix(uint32_t& a, uint32_t& b, uint32_t& c) {  
    a-=c; a^=rot32(c,4);  c+=b;  
    b-=a; b^=rot32(a,6);  a+=c;  
    c-=b; c^=rot32(b,8);  b+=a;  
    a-=c; a^=rot32(c,16); c+=b;  
    b-=a; b^=rot32(a,19); a+=c;  
    c-=b; c^=rot32(b,4);  b+=a;  
}  
  
static inline void nh_final(uint32_t& a, uint32_t& b, uint32_t& c) {  
    c^=b; c-=rot32(b,14);  
    a^=c; a-=rot32(c,11);  
    b^=a; b-=rot32(a,25);  
    c^=b; c-=rot32(b,16);  
    a^=c; a-=rot32(c,4);  
    b^=a; b-=rot32(a,14);  
    c^=b; c-=rot32(b,24);  
}  
  
static int32_t nhash_bytes(const void* key, size_t length, int32_t initval) {  
    const uint8_t* k = (const uint8_t*)key;  
    uint32_t a = 0xdeadbeef + (uint32_t)length + (uint32_t)initval, b = a, c = a;  
    while (length > 12) {  
        a += k[0] | ((uint32_t)k[1]<<8) | ((uint32_t)k[2]<<16) | ((uint32_t)k[3]<<24);  
        b += k[4] | ((uint32_t)k[5]<<8) | ((uint32_t)k[6]<<16) | ((uint32_t)k[7]<<24);  
        c += k[8] | ((uint32_t)k[9]<<8) | ((uint32_t)k[10]<<16)| ((uint32_t)k[11]<<24);  
        nh_mix(a,b,c); k += 12; length -= 12;  
    }  
    switch (length) {  
        case 12: c += (uint32_t)k[11]<<24; /* fall through */  
        case 11: c += (uint32_t)k[10]<<16; /* fall through */  
        case 10: c += (uint32_t)k[9]<<8;   /* fall through */  
        case 9:  c += k[8];                /* fall through */  
        case 8:  b += (uint32_t)k[7]<<24;  /* fall through */  
        case 7:  b += (uint32_t)k[6]<<16;  /* fall through */  
        case 6:  b += (uint32_t)k[5]<<8;   /* fall through */  
        case 5:  b += k[4];                /* fall through */  
        case 4:  a += (uint32_t)k[3]<<24;  /* fall through */  
        case 3:  a += (uint32_t)k[2]<<16;  /* fall through */  
        case 2:  a += (uint32_t)k[1]<<8;   /* fall through */  
        case 1:  a += k[0]; break;  
        case 0:  return (int32_t)(c & 32767);  
    }  
    nh_final(a,b,c);  
    return (int32_t)(c & 32767);  
}  
  
// hash() из packjt77.f90: nhash(s, len, 146) & 32767  
static int jtty_hash15(const char* s, size_t len) {  
    return (int)nhash_bytes(s, len, 146);  
}  
  
// ============================================================  
// ihashcall — 22/12/10-битный хэш позывного  
// ============================================================  
static int ihashcall(const char* c0_13, int m) {  
    static const char C38[] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ/";  
    uint64_t n8 = 0;  
    char buf[12];  
    memset(buf, ' ', 11); buf[11] = 0;  
    size_t l = strlen(c0_13); if (l > 11) l = 11;  
    memcpy(buf, c0_13, l);  
    for (int i = 0; i < 11; i++) {  
        const char* p = strchr(C38, buf[i]);  
        n8 = 38 * n8 + (uint64_t)(p ? (p - C38) : 0);  
    }  
    uint64_t prod = 47055833459ULL * n8;  
    if (m <= 0) return 0;  
    return (int)(prod >> (64 - m));  
}  
  
// ============================================================  
// pack28 — точный порт pack28() из packjt77.f90 (WSJT-X 3.3)  
// ============================================================  
#define PACK28_NTOKENS  2063592L  
#define PACK28_MAX22    4194304L  
  
static const char P28_A1[38] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"; // 37  
static const char P28_A2[37] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";  // 36  
static const char P28_A3[11] = "0123456789";                            // 10  
static const char P28_A4[28] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ";           // 27  
  
static int idx_in(const char* tab, char c) {  
    const char* p = strchr(tab, c);  
    return p ? (int)(p - tab) : -1;  
}  
  
static bool is_dig(char c)  { return c >= '0' && c <= '9'; }  
static bool is_let(char c)  { return c >= 'A' && c <= 'Z'; }  
  
static int32_t pack28(const char* call) {  
    char c13[14];  
    memset(c13, ' ', 13);  
    c13[13] = 0;  
    size_t L = strlen(call);  
    if (L > 13) L = 13;  
    memcpy(c13, call, L);  
  
    if (!strncmp(c13, "3DA0", 4)) {  
        c13[2] = '0';  
        memmove(c13 + 3, c13 + 4, 9);  
    }  
    if (c13[0] == '3' && c13[1] == 'X' && is_let(c13[2])) {  
        c13[0] = 'Q';  
    }  
  
    int n = 13;  
    while (n > 0 && c13[n - 1] == ' ') n--;  
  
    if (n >= 2 && !strncmp(c13, "DE", 2) && (n == 2 || c13[2] == ' '))  
        return 0;  
    if (n >= 3 && !strncmp(c13, "QRZ", 3) && (n == 3 || c13[3] == ' '))  
        return 1;  
    if (n >= 2 && !strncmp(c13, "CQ", 2) && (n == 2 || c13[2] == ' '))  
        return 2;  
  
    if (n >= 4 && !strncmp(c13, "CQ ", 3)) {  
        bool digits = true;  
        int nd = 0, val = 0;  
        for (int i = 3; i < n; i++) {  
            if (!is_dig(c13[i])) { digits = false; break; }  
            val = val * 10 + (c13[i] - '0');  
            nd++;  
        }  
        if (digits && nd >= 1 && nd <= 3)  
            return (3 + val) & 0x0FFFFFFFL;  
  
        if (n == 8) {  
            bool lets = true;  
            for (int i = 4; i < 8; i++)  
                if (!is_let(c13[i])) { lets = false; break; }  
            if (lets)  
                return (3 + 1000 + jtty_hash15(c13 + 4, 4)) & 0x0FFFFFFFL;  
        }  
    }  
    if (n == 8 && !strncmp(c13, "QRZ ", 4)) {  
        bool lets = true;  
        for (int i = 4; i < 8; i++)  
            if (!is_let(c13[i])) { lets = false; break; }  
        if (lets)  
            return (3 + 1000 + jtty_hash15(c13 + 4, 4)) & 0x0FFFFFFFL;  
    }  
  
    int iarea1 = 0;  
    for (int i = n - 1; i >= 0; i--) {  
        if (is_dig(c13[i])) { iarea1 = i + 1; break; }  
    }  
    if (iarea1 != 2 && iarea1 != 3) {  
        int32_t n22 = ihashcall(c13, 22);  
        return (PACK28_NTOKENS + n22) & 0x0FFFFFFFL;  
    }  
  
    if (n > 6) {  
        int32_t n22 = ihashcall(c13, 22);  
        return (PACK28_NTOKENS + n22) & 0x0FFFFFFFL;  
    }  
  
    char cs[7];  
    memset(cs, ' ', 6);  
    cs[6] = 0;  
    if (iarea1 == 2) {  
        cs[0] = ' ';  
        memcpy(cs + 1, c13, 5);  
    } else {  
        memcpy(cs, c13, 6);  
    }  
  
    int i1 = idx_in(P28_A1, cs[0]);  
    int i2 = idx_in(P28_A2, cs[1]);  
    int i3 = idx_in(P28_A3, cs[2]);  
    int i4 = idx_in(P28_A4, cs[3]);  
    int i5 = idx_in(P28_A4, cs[4]);  
    int i6 = idx_in(P28_A4, cs[5]);  
  
    if (i1 < 0 || i2 < 0 || i3 < 0 || i4 < 0 || i5 < 0 || i6 < 0) {  
        int32_t n22 = ihashcall(c13, 22);  
        return (PACK28_NTOKENS + n22) & 0x0FFFFFFFL;  
    }  
  
    int64_t n28 = 36LL*10*27*27*27*i1 + 10LL*27*27*27*i2 + 27LL*27*27*i3  
                + 27LL*27*i4 + 27LL*i5 + i6;  
    n28 += (int64_t)PACK28_NTOKENS + PACK28_MAX22;  
    return (int32_t)(n28 & 0x0FFFFFFFL);  
}  
  
// ============================================================  
// Атомы -> 32-битное тело -> 34-битный payload  
// ============================================================  
static inline uint32_t finish_struct(uint32_t body, int family) {  
    return 4u * (8u * body + (uint32_t)family) + 2u;  
}  
  
static bool grid4_to_index(const char* g, int* idx) {  
    if (!g[0] || !g[1] || !g[2] || !g[3]) return false;  
    if (g[0] < 'A' || g[0] > 'R' || g[1] < 'A' || g[1] > 'R') return false;  
    if (!isdigit((unsigned char)g[2]) || !isdigit((unsigned char)g[3])) return false;  
    *idx = ((g[0]-'A')*18 + (g[1]-'A'))*100 + (g[2]-'0')*10 + (g[3]-'0');  
    return true;  
}  
  
static bool pos_incomplete(int len, int na);  
  
static bool standard_call(const char* call) {  
    return pack28(call) > (int32_t)(PACK28_NTOKENS + PACK28_MAX22);  
}  
  
static bool jtty_pack_atom(const JttyAtom* atom, bool eom, uint64_t* payload34) {  
    uint32_t n32 = 0;  
    switch (atom->kind) {  
    case JTTY_ATOM_CALL: {  
        if (atom->subtype < 0 || atom->subtype > 5) return false;  
        if (!standard_call(atom->text)) return false;  
        int32_t n28 = pack28(atom->text);  
        if (n28 < 0) return false;  
        int i2, n2;  
        if (atom->subtype <= 3) { i2 = 0; n2 = atom->subtype; }  
        else                    { i2 = 1; n2 = atom->subtype - 4; }  
        n32 = ((uint32_t)n28 << 4) | ((uint32_t)n2 << 2) | (uint32_t)i2;  
        break;  
    }  
    case JTTY_ATOM_EXCH_NUM: {  
        if (atom->role < 0 || atom->role > 1) return false;  
        if (atom->subtype < 0 || atom->subtype > 15) return false;  
        if (atom->value < 0 || atom->value > 131071) return false;  
        uint32_t body = ((uint32_t)atom->role << 26) | ((uint32_t)atom->subtype << 22)  
                      | ((uint32_t)atom->value << 5);  
        n32 = finish_struct(body, 0);  
        break;  
    }  
    case JTTY_ATOM_EXCH_LOC: {  
        if (atom->role < 0 || atom->role > 1) return false;  
        uint32_t body = ((uint32_t)atom->role << 26) | ((uint32_t)atom->subtype << 22)  
                      | ((uint32_t)(atom->value & 0x1FFFF) << 5);  
        n32 = finish_struct(body, 1);  
        break;  
    }  
    case JTTY_ATOM_EXCH_PAIR: {  
        uint32_t body = ((uint32_t)atom->subtype << 22)  
                      | ((uint32_t)(atom->value & 0x3FF) << 12)  
                      | ((uint32_t)(atom->value2 & 0x3FF) << 2);  
        n32 = finish_struct(body, 2);  
        break;  
    }  
    case JTTY_ATOM_EXCH_NUM_TIME: {  
        uint32_t body = ((uint32_t)atom->role << 26) | ((uint32_t)atom->subtype << 22)  
                      | ((uint32_t)(atom->value & 0x1FFFF) << 5);  
        n32 = finish_struct(body, 3);  
        break;  
    }  
    case JTTY_ATOM_CONTROL: {  
        if (atom->subtype < 0 || atom->subtype > 17) return false;  
        uint32_t body = (0u << 23) | ((uint32_t)atom->subtype << 16);  
        n32 = finish_struct(body, 4);  
        break;  
    }  
    case JTTY_ATOM_GRID4: {  
        if (atom->role < 0 || atom->role > 1) return false;  
        int gi;  
        if (!grid4_to_index(atom->text, &gi)) return false;  
        uint32_t data = ((uint32_t)atom->role << 22) | ((uint32_t)gi << 7);  
        uint32_t body = (1u << 23) | data;  
        n32 = finish_struct(body, 4);  
        break;  
    }  
    case JTTY_ATOM_TEXT5: {  
        uint32_t top30 = 0;  
        for (int i = 0; i < 5; i++) {  
            char c = atom->text[i] ? atom->text[i] : ' ';  
            int ix = source_index(c);  
            if (ix < 0) return false;  
            top30 = 64u * top30 + (uint32_t)ix;  
        }  
        n32 = 4u * top30 + 3u;  
        break;  
    }  
    default:  
        return false;  
    }  
    *payload34 = ((uint64_t)n32 << 2) | (eom ? 1ULL : 0ULL);  
    return true;  
}  
  
static bool jtty_pack_atoms(const JttyAtom* atoms, int natoms,  
                            uint64_t* frames, int* nframes, int is_final) {  
    bool final_ok = (is_final < 0) || (is_final != 0);  
    *nframes = 0;  
    if (natoms < 1 || natoms > JTTY_MAX_FRAMES) return false;  
    for (int i = 0; i < natoms; i++) {  
        bool eom = (i == natoms - 1) && final_ok;  
        if (!jtty_pack_atom(&atoms[i], eom, &frames[i])) return false;  
    }  
    *nframes = natoms;  
    return true;  
}  
  
// ============================================================  
// Нормализация и минимальный пакер сообщения  
// ============================================================  
static void normalize_jtty(char* s) {  
    for (char* p = s; *p; p++) {  
        *p = (char)toupper((unsigned char)*p);  
        if (*p == '~') *p = ' ';  
        if (source_index(*p) < 0) *p = '#';  
    }  
    char* r = s; char* w = s; bool sp = true;  
    while (*r) {  
        if (*r == ' ') { if (!sp) *w++ = ' '; sp = true; }  
        else { *w++ = *r; sp = false; }  
        r++;  
    }  
    if (w > s && w[-1] == ' ') w--;  
    *w = 0;  
}  
  
static bool is_standard_callsign(const char* s) {  
    return pack28(s) > (int32_t)(PACK28_NTOKENS + PACK28_MAX22);  
}  
  
static bool jtty_pack_message(const char* umsg_in, JttyFrameList* fl,  
                              int exchange_profile, int is_final) {  
    (void)exchange_profile;  
    char msg[81]; strncpy(msg, umsg_in, 80); msg[80] = 0;  
    normalize_jtty(msg);  
    if (!msg[0]) return false;  
  
    JttyAtom atoms[JTTY_MAX_FRAMES];  
    int na = 0;  
  
    char buf[81]; strcpy(buf, msg);  
    char* w[8]; int nw = 0;  
    for (char* t = strtok(buf, " "); t && nw < 8; t = strtok(NULL, " ")) w[nw++] = t;  
  
    if (nw >= 2 && strcmp(w[0], "CQ") == 0 && is_standard_callsign(w[1])) {  
        atoms[na].kind = JTTY_ATOM_CALL; atoms[na].subtype = JTTY_CALL_CQ;  
        atoms[na].role = JTTY_ROLE_FIELD_ONLY;  
        strncpy(atoms[na].text, w[1], 13); atoms[na].text[13] = 0;  
        na++;  
        int consumed = 3 + (int)strlen(w[1]);  
        char* rest = msg + consumed; while (*rest == ' ') rest++;  
        if (strncmp(rest, "CQ", 2) == 0 && (rest[2] == 0 || rest[2] == ' ')) {  
            rest += 2; while (*rest == ' ') rest++;  
        }  
        memmove(msg, rest, strlen(rest) + 1);  
    } else if (nw >= 1 && is_standard_callsign(w[0]) && nw <= 2) {  
        int subtype = JTTY_CALL_CALL;  
        if (nw == 2 && strcmp(w[1], "TU") == 0) subtype = JTTY_CALL_CALL_TU;  
        atoms[na].kind = JTTY_ATOM_CALL; atoms[na].subtype = subtype;  
        atoms[na].role = JTTY_ROLE_FIELD_ONLY;  
        strncpy(atoms[na].text, w[0], 13); atoms[na].text[13] = 0;  
        na++;  
        msg[0] = 0;  
    }  
  
    int len = (int)strlen(msg);  
    for (int pos = 0; pos < len && na < JTTY_MAX_FRAMES; pos += 5) {  
        JttyAtom* a = &atoms[na];  
        memset(a, 0, sizeof(*a));  
        a->kind = JTTY_ATOM_TEXT5; a->role = JTTY_ROLE_FIELD_ONLY;  
        int k = 0;  
        for (int i = pos; i < pos + 5 && i < len; i++) a->text[k++] = msg[i];  
        while (k < 5) a->text[k++] = ' ';  
        a->text[5] = 0;  
        na++;  
    }  
    if (na == 0) return false;  
    if (len > 0 && na >= JTTY_MAX_FRAMES && pos_incomplete(len, na)) return false;  
  
    return jtty_pack_atoms(atoms, na, fl->frames, &fl->nframes, is_final);  
}  
static bool pos_incomplete(int len, int na) { return len > na * 5; }  
  
// ============================================================  
// FEC: CRC-12(0x80F) + tail-biting conv K=10 (0o1167/0o1545)  
// ============================================================  
#define JTTY_PAYLOAD_BITS 34  
#define JTTY_INFO_BITS    46          // 34 + 12  
#define JTTY_NU           9  
#define JTTY_G0           01167       // octal  
#define JTTY_G1           01545  
#define JTTY_CRC_POLY     0x80F  
  
static inline int parity32(uint32_t x) {  
    x ^= x >> 16; x ^= x >> 8; x ^= x >> 4; x ^= x >> 2; x ^= x >> 1;  
    return x & 1;  
}  
  
static uint16_t jtty_crc12(const uint8_t* info_bits /*34*/) {  
    uint16_t reg = 0;  
    for (int i = 0; i < JTTY_PAYLOAD_BITS; i++) {  
        uint16_t fb = ((reg >> 11) ^ info_bits[i]) & 1;  
        reg = (uint16_t)(((reg << 1) & 0xFFF) ^ (fb ? JTTY_CRC_POLY : 0));  
    }  
    return reg;  
}  
  
// payload34 -> itone[59]: is13 + 46 дибитов  
static void jtty_build_frame(uint64_t payload34, uint8_t* itone /*59*/) {  
    uint8_t info[JTTY_INFO_BITS];  
    for (int i = 0; i < JTTY_PAYLOAD_BITS; i++)  
        info[i] = (uint8_t)((payload34 >> (33 - i)) & 1);  
  
    uint16_t crc = jtty_crc12(info);  
    for (int i = 0; i < 12; i++)  
        info[JTTY_PAYLOAD_BITS + i] = (uint8_t)((crc >> (11 - i)) & 1);  
  
    uint16_t reg10 = 0;  
    for (int t = 0; t < JTTY_NU; t++)  
        reg10 = (uint16_t)((reg10 << 1) | info[JTTY_INFO_BITS - JTTY_NU + t]);  
  
    for (int i = 0; i < JTTY_NSYNC; i++) itone[i] = JTTY_IS13[i];  
  
    for (int i = 0; i < JTTY_INFO_BITS; i++) {  
        reg10 = (uint16_t)(((reg10 << 1) | info[i]) & 0x3FF);  
        uint8_t b0 = (uint8_t)parity32(reg10 & JTTY_G0);  
        uint8_t b1 = (uint8_t)parity32(reg10 & JTTY_G1);  
        static const uint8_t GRAY4[4] = {0, 1, 3, 2};  
        itone[JTTY_NSYNC + i] = GRAY4[(b0 << 1) | b1];  
    }  
}  
  
// ============================================================  
// Модулятор: NCO + очередь кадров + символьный автомат  
// ============================================================  
static int16_t  tx_lut[JTTYTX_LUT_LEN];          // Q15 синус  
static uint32_t nco_step_tone[JTTY_NTONES];      // шаги фазы 4 тонов  
static volatile uint32_t nco_step;               // текущий шаг (Core 1)  
static uint32_t nco_phase;                       // аккумулятор фазы (непрерывный)  
  
// Длительность символа в отсчётах, Q16: 15464*0.032 = 494.848  
static uint32_t sym_len_q16;  
static uint32_t sym_acc_q16;                     // накопитель дробной части  
  
// Очередь кадров: готовые itone[59] на кадр  
static uint8_t  tx_frames[JTTY_MAX_FRAMES][JTTY_NSYMS];  
static volatile uint8_t tx_nframes;              // кадров в очереди  
static volatile uint8_t tx_fpos;                 // текущий кадр  
static uint8_t  sym_idx;                         // текущий символ в кадре 0..58  
  
// Эхо-строка исходного текста для OLED  
#define JTTYTX_TEXT_LEN 82  
static char     tx_echo[JTTYTX_TEXT_LEN];  
static volatile uint8_t tx_elen;  
static volatile uint8_t tx_epos;  
  
// Символьный слот: Core 0 кладёт (тон, длина), Core 1 подхватывает  
// на границе символа — поток непрерывный.  
static volatile uint8_t  tx_pend_tone;           // ожидающий тон 0..3  
static volatile uint32_t tx_pend_len;            // его длительность в отсчётах  
static volatile bool     tx_pend_valid;          // слот занят  
static volatile uint32_t tx_sym_rem;             // отсчётов до конца символа  
static volatile bool     tx_active;              // идёт передача  
  
static bool tail_tone;                           // хвост f0 перед концом  
  
// ---------------------------------------------------------------------------  
// Core 1: следующий отсчёт. CPFSK — фаза непрерывна, меняется только шаг.  
// ---------------------------------------------------------------------------  
int16_t __not_in_flash_func(jttytx_sample)(void) {  
    if (!tx_active) return 0;  
    if (tx_sym_rem == 0) {  
        if (tx_pend_valid) {                     // граница символа — подхватить слот  
            tx_sym_rem    = tx_pend_len;  
            nco_step      = nco_step_tone[tx_pend_tone & 3];  
            tx_pend_valid = false;  
        } else {  
            // Core 0 ещё не подал символ — держим текущий тон, фаза крутится  
            nco_phase += nco_step;  
            return tx_lut[(nco_phase >> 24) & (JTTYTX_LUT_LEN - 1)];  
        }  
    }  
    tx_sym_rem--;  
    nco_phase += nco_step;  
    return tx_lut[(nco_phase >> 24) & (JTTYTX_LUT_LEN - 1)];  
}  
  
// ---------------------------------------------------------------------------  
// Core 0: подача одного символа в pending-слот.  
// Длина — из накопителя Q16 (494/495 отсчётов, среднее 494.848).  
// ---------------------------------------------------------------------------  
static void feed_tone(uint8_t tone) {  
    sym_acc_q16 += sym_len_q16;  
    tx_pend_tone  = tone;  
    tx_pend_len   = sym_acc_q16 >> 16;  
    sym_acc_q16  &= 0xFFFF;  
    tx_pend_valid = true;                        // Core 1 заберёт на границе  
}  
  
// ---------------------------------------------------------------------------  
// Core 0: символьный автомат. Вызывать из loop() каждую итерацию.  
// ---------------------------------------------------------------------------  
void jttytx_poll(void) {  
    if (!tx_active) return;  
    if (tx_pend_valid) return;                   // слот занят — запас ~1 символ  
  
    // символы текущего кадра  
    if (sym_idx < JTTY_NSYMS) {  
        feed_tone(tx_frames[tx_fpos][sym_idx++]);  
        return;  
    }  
  
    // следующий кадр из очереди  
    if (tx_fpos + 1 < tx_nframes) {  
        tx_fpos++;  
        sym_idx = 0;  
        // эхо на OLED: маркер кадра  
        af_oled_putc('|');  
        return;  
    }  
  
    // очередь пуста — хвост нижним тоном ~100 мс, потом конец передачи  
    if (!tail_tone) {  
        tail_tone = true;  
        feed_tone(0);  
        // переопределить длину хвоста: ровно 100 мс  
        tx_pend_len = (AF_FS_OUT_HZ * 100u) / 1000u;  
    } else if (tx_sym_rem == 0 && !tx_pend_valid) {  
        tx_active = false;                       // хвост доигран Core 1 — гасим  
    }  
}  
  
// ---------------------------------------------------------------------------  
// Core 0: API  
// ---------------------------------------------------------------------------  
#define JTTYTX_AMP  14000   // ~0.43 FS — как у RTTYTX, запас под FIR и клиппер  
  
void jttytx_init(void) {  
    for (uint32_t i = 0; i < JTTYTX_LUT_LEN; i++)  
        tx_lut[i] = (int16_t)(sinf(2.0f * (float)M_PI * (float)i / JTTYTX_LUT_LEN) * (float)JTTYTX_AMP);  
    for (int t = 0; t < JTTY_NTONES; t++) {  
        uint32_t f_mhz = JTTY_F0_MILLIHZ + (uint32_t)t * JTTY_BAUD_MILLIHZ;  
        nco_step_tone[t] = (uint32_t)(((uint64_t)f_mhz << 32) / (AF_FS_OUT_HZ * 1000ULL));  
    }  
    nco_step  = nco_step_tone[0];  
    nco_phase = 0;  
    // 32 мс в отсчётах, Q16  
    sym_len_q16 = (uint32_t)(((uint64_t)AF_FS_OUT_HZ * 32ULL * 65536ULL) / 1000ULL);  
    sym_acc_q16 = 0;  
    tx_active = false;  
    tx_nframes = tx_fpos = 0;  
    sym_idx = JTTY_NSYMS;  
    tx_elen = tx_epos = 0;  
    tx_sym_rem = 0;  
    tx_pend_valid = false;  
    tail_tone = false;  
    Serial.printf("[JTTYTX] init f0=%lu.%03lu Hz sym=%lu.%03lu smp\n",  
                  (unsigned long)(JTTY_F0_MILLIHZ / 1000),  
                  (unsigned long)(JTTY_F0_MILLIHZ % 1000),  
                  (unsigned long)(sym_len_q16 >> 16),  
                  (unsigned long)(((sym_len_q16 & 0xFFFF) * 1000UL) >> 16));  
}  
  
bool jttytx_send(const char* text) {  
    if (tx_active) return false;  
    if (!text || !*text) return false;  
  
    // Упаковка: при ошибке TX не поднимаем вообще  
    JttyFrameList fl;  
    if (!jtty_pack_message(text, &fl, JTTY_EXCHANGE_UNKNOWN, -1)) {  
        Serial.println(F("[JTTYTX] упаковка сообщения не удалась"));  
        return false;  
    }  
    if (fl.nframes < 1 || fl.nframes > JTTY_MAX_FRAMES) return false;  
  
    // Кадры -> itone[59] в очередь  
    for (int f = 0; f < fl.nframes; f++)  
        jtty_build_frame(fl.frames[f], tx_frames[f]);  
  
    // Эхо-строка для OLED (нормализованный текст)  
    uint8_t e = 0;  
    for (uint8_t i = 0; text[i] && e < JTTYTX_TEXT_LEN - 1; i++)  
        tx_echo[e++] = (char)toupper((unsigned char)text[i]);  
    tx_elen = e; tx_epos = 0;  
  
    tx_nframes = (uint8_t)fl.nframes;  
    tx_fpos = 0;  
    sym_idx = 0;  
    sym_acc_q16 = 0;  
    tx_sym_rem = 0;  
    tx_pend_valid = false;  
    tail_tone = false;  
    af_oled_putc('>');                           // префикс TX на экране, как у CW/RTTY  
    tx_active = true;  
    return true;  
}  
  
bool jttytx_busy(void) { return tx_active; }  
  
uint32_t jttytx_center_hz(void) { return JTTYTX_CENTER_HZ; }  
  
void jttytx_abort(void) {  
    tx_active = false;  
    tx_sym_rem = 0;  
    tx_pend_valid = false;  
    tx_nframes = tx_fpos = 0;  
    sym_idx = JTTY_NSYMS;  
    tx_elen = tx_epos = 0;  
    tail_tone = false;  
}  
  
// ============================================================  
// jttytx_selftest — проверка кодека по эталонным векторам WSJT-X.  
// ============================================================  
void jttytx_selftest(void) {  
    Serial.println(F("\n[JTTY TEST] start"));  
  
    struct { const char* call; int h10, h12, h22; } hv[] = {  
        { "K1ABC",       712, 2851, 2920267 },  
        { "W9XYZ",       972, 3889, 3982604 },  
        { "PJ4/K1ABC",   346, 1387, 1420834 },  
        { "KH1/KH7Z",    201,  806,  825805 },  
        { "K1JT/P",      959, 3839, 3931158 },  
        { "W3CCX",       665, 2662, 2726483 },  
        { "3DA0ABC",      48,  192,  196695 },  
        { "3XABC",       563, 2254, 2308721 },  
        { "///////////", 671, 2685, 2749801 },  
        { "ZZZZZZZZZZZ", 902, 3609, 3695718 },  
        { "99999999999", 762, 3050, 3123740 },  
        { "           ",   0,    0,       0 },  
    };  
    int pass = 0, total = 0;  
    Serial.println(F("--- ihashcall ---"));  
    for (unsigned i = 0; i < sizeof(hv)/sizeof(hv[0]); i++) {  
        int g10 = ihashcall(hv[i].call, 10);  
        int g12 = ihashcall(hv[i].call, 12);  
        int g22 = ihashcall(hv[i].call, 22);  
        bool ok = (g10 == hv[i].h10 && g12 == hv[i].h12 && g22 == hv[i].h22);  
        total++; pass += ok;  
        Serial.printf("  %-12s 10:%4d 12:%4d 22:%7d  %s\n",  
                      hv[i].call, g10, g12, g22, ok ? "OK" : "FAIL");  
    }  
  
    Serial.println(F("--- pack28 ---"));  
    struct { const char* s; int32_t lo, hi; } pv[] = {  
        { "DE",        0, 0 }, { "QRZ", 1, 1 }, { "CQ", 2, 2 },  
        { "K1ABC",     PACK28_NTOKENS + PACK28_MAX22, 0x0FFFFFFFL },  
        { "RU0AOG",    PACK28_NTOKENS + PACK28_MAX22, 0x0FFFFFFFL },  
        { "PJ4/K1ABC", PACK28_NTOKENS + 1420834, PACK28_NTOKENS + 1420834 },  
    };  
    for (unsigned i = 0; i < sizeof(pv)/sizeof(pv[0]); i++) {  
        int32_t n28 = pack28(pv[i].s);  
        bool ok = (n28 >= pv[i].lo && n28 <= pv[i].hi);  
        total++; pass += ok;  
        Serial.printf("  pack28(%-10s) = %9ld  %s\n", pv[i].s, (long)n28, ok ? "OK" : "FAIL");  
    }  
  
    Serial.println(F("--- pack atom ---"));  
    {  
        JttyAtom a; memset(&a, 0, sizeof a);  
        a.kind = JTTY_ATOM_TEXT5; strncpy(a.text, "HELLO", sizeof(a.text) - 1);  
        uint64_t p34 = 0;  
        bool ok = jtty_pack_atom(&a, true, &p34) && (p34 == 0x11395558DULL);  
        total++; pass += ok;  
        Serial.printf("  TEXT5 'HELLO' payload=0x%09llX  %s (expect 0x11395558D)\n",  
                      (unsigned long long)p34, ok ? "OK" : "FAIL");  
    }  
    {  
        JttyAtom a; memset(&a, 0, sizeof a);  
        a.kind = JTTY_ATOM_CONTROL; a.subtype = JTTY_CONTROL_AGN;  
        uint64_t p34 = 0;  
        bool ok = jtty_pack_atom(&a, true, &p34) && (p34 == 0x000000049ULL);  
        total++; pass += ok;  
        Serial.printf("  CONTROL 'AGN?' payload=0x%09llX  %s (expect 0x000000049)\n",  
                      (unsigned long long)p34, ok ? "OK" : "FAIL");  
    }  
  
    Serial.println(F("--- frame ---"));  
    {  
        uint8_t itone[JTTY_NSYMS];  
        jtty_build_frame(0x11395558DULL, itone);  
        bool ok = true;  
        for (int i = 0; i < JTTY_NSYNC; i++) if (itone[i] != JTTY_IS13[i]) ok = false;  
        total++; pass += ok;  
        Serial.print(F("  sync+data: "));  
        for (int i = 0; i < JTTY_NSYMS; i++) Serial.printf("%d", itone[i]);  
        Serial.printf("  sync:%s\n", ok ? "OK" : "FAIL");  
    }  
  
    Serial.printf("[JTTY TEST] %d/%d passed\n\n", pass, total);  
}
