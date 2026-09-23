from dataclasses import dataclass

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def frame(body: str) -> bytes:
    return f"${body},C{crc16(body.encode()):04X}#".encode()

class StreamParser:
    def __init__(self, maximum=256): self.buf=bytearray(); self.maximum=maximum
    def feed(self, chunk: bytes):
        out=[]
        for b in chunk:
            if b==36: self.buf=bytearray((b,))
            elif self.buf:
                self.buf.append(b)
                if len(self.buf)>self.maximum: self.buf.clear()
                elif b==35: out.append(bytes(self.buf)); self.buf.clear()
        return out

def verify(raw: bytes) -> bool:
    if not (raw.startswith(b"$") and raw.endswith(b"#")): return False
    marker=raw.rfind(b",C")
    if marker<0 or len(raw)-marker!=7: return False
    try: expected=int(raw[marker+2:-1],16)
    except ValueError: return False
    return crc16(raw[1:marker])==expected

@dataclass(frozen=True)
class Pid: ap:float; ad:float; vp:float; vi:float; tp:float; td:float

def prepare(trial:int, ttl_ms:int, pid:Pid)->bytes:
    return frame(f"P1,PREP,{trial},{ttl_ms},{pid.ap:.2f},{pid.ad:.2f},{pid.vp:.2f},{pid.vi:.2f},{pid.tp:.2f},{pid.td:.2f}")

def manual_set(pid:Pid)->bytes:
    return frame(f"P1,MSET,{pid.ap:.2f},{pid.ad:.2f},{pid.vp:.2f},{pid.vi:.2f},{pid.tp:.2f},{pid.td:.2f}")

def motor_diag(action:str)->bytes:
    if action not in {"L+","L-","R+","R-","STOP"}: raise ValueError(action)
    return frame(f"P1,DIAG,{action}")
