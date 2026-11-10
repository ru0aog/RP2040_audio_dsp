import os, wave, struct  
  
os.chdir(os.path.dirname(os.path.abspath(__file__)))
  
w = wave.open('voice.wav', 'rb')  
n = w.getnframes()  
data = struct.unpack(f'<{n}h', w.readframes(n))  
w.close()  
  
# Линейный ресемплинг 16000 -> 15625  
ratio = 15625 / 16000  
out_n = int(n * ratio)  
out = []  
for i in range(out_n):  
    pos = i / ratio  
    i0 = int(pos)  
    i1 = min(i0 + 1, n - 1)  
    frac = pos - i0  
    s = int(data[i0] * (1 - frac) + data[i1] * frac)  
    s = int(s * 0.6)          # громкость ~60%, чтобы не клипповать  
    out.append(max(-32768, min(32767, s)))  
  
with open('clip_voice.inc', 'w') as f:  
    f.write('static const int16_t clip_voice[] = {\n')  
    for i in range(0, len(out), 12):  
        f.write('    ' + ','.join(str(x) for x in out[i:i+12]) + ',\n')  
    f.write('};\n')  
  
print(f'OK: {n} -> {out_n} samples ({out_n/15625:.2f} s)')
