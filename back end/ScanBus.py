"""Servo bus diagnostic + ID fixer for the SO-101 (Feetech STS3215 on COM4).

    python ScanBus.py              scan: which IDs answer, and how cleanly
    python ScanBus.py --set-id 2   with ONLY the wrongly-addressed servo on the
                                   bus (unplug the rest), re-address the single
                                   ID-1 servo to the given ID and verify

Two servos sharing an ID show up as: garbled replies on that ID, silence on the
ID the second servo should have. Fix = isolate the stray servo and re-address it.
"""
import sys, time
import scservo_sdk as scs

PORT, BAUD = "COM4", 1_000_000
ADDR_ID, ADDR_LOCK = 5, 55
JOINTS = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex", 4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}


def OpenBus():
    Ph, Pk = scs.PortHandler(PORT), scs.PacketHandler(0)
    if not Ph.openPort() or not Ph.setBaudRate(BAUD):
        sys.exit(f"Cannot open {PORT} at {BAUD}")
    Ph.setPacketTimeout(30)
    return Ph, Pk


def Scan(Ph, Pk, Rounds=10):
    print(f"Scanning IDs 0-253 on {PORT} @ {BAUD}...")
    Found = [i for i in range(254) if Pk.ping(Ph, i)[1] == scs.COMM_SUCCESS]
    print("Responding IDs:", Found or "none")
    print(f"\nReliability over {Rounds} pings per expected joint:")
    Bad = []
    for Id, Name in JOINTS.items():
        Results = [Pk.ping(Ph, Id)[1] for _ in range(Rounds)]
        Ok = Results.count(scs.COMM_SUCCESS)
        Corrupt = Results.count(scs.COMM_RX_CORRUPT)
        Verdict = "OK" if Ok == Rounds else ("COLLISION? (garbled replies)" if Corrupt else "SILENT")
        if Ok != Rounds:
            Bad.append(Id)
        print(f"  ID {Id} {Name:<14} {Ok:>2}/{Rounds} clean, {Corrupt} corrupt   {Verdict}")
    return Found, Bad


def SetId(Ph, Pk, NewId):
    Found, _ = Scan(Ph, Pk, Rounds=20)
    if Found != [1]:
        sys.exit(f"\nRefusing: expected exactly one servo (ID 1) on the bus, saw {Found}. Unplug the others and retry.")
    Clean = sum(Pk.ping(Ph, 1)[1] == scs.COMM_SUCCESS for _ in range(20))
    if Clean < 20:
        sys.exit(f"\nRefusing: ID 1 still replies unreliably ({Clean}/20); more than one servo may still be attached.")
    print(f"\nRe-addressing the lone servo 1 -> {NewId} ({JOINTS.get(NewId, '?')})")
    Pk.write1ByteTxRx(Ph, 1, ADDR_LOCK, 0)
    Res, Err = Pk.write1ByteTxRx(Ph, 1, ADDR_ID, NewId)
    time.sleep(0.1)
    Pk.write1ByteTxRx(Ph, NewId, ADDR_LOCK, 1)
    Model, Res, Err = Pk.ping(Ph, NewId)
    Old = Pk.ping(Ph, 1)[1] == scs.COMM_SUCCESS
    if Res == scs.COMM_SUCCESS and not Old:
        print(f"Done: ID {NewId} answers (model {Model}), ID 1 is now silent. Plug the chain back together and rerun the scan.")
    else:
        sys.exit(f"ID change not confirmed (new id ping res={Res}, old id still answers={Old}). Power-cycle and rescan.")


if __name__ == "__main__":
    Ph, Pk = OpenBus()
    try:
        if len(sys.argv) == 3 and sys.argv[1] == "--set-id":
            SetId(Ph, Pk, int(sys.argv[2]))
        else:
            Scan(Ph, Pk)
    finally:
        Ph.closePort()
