#!/usr/bin/env python3
import os
import csv
import argparse
import random

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True, help="Path to data.csv")
    ap.add_argument("--out_dir", required=True)
    ap.add_argument("--val", type=float, default=0.15)
    ap.add_argument("--test", type=float, default=0.15)
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    random.seed(args.seed)
    rows = []
    with open(args.csv, "r") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append({"file_path": r["file_path"], "yaw": r["yaw"]})

    random.shuffle(rows)
    n = len(rows)
    n_test = int(n * args.test)
    n_val = int(n * args.val)
    test_rows = rows[:n_test]
    val_rows = rows[n_test:n_test+n_val]
    train_rows = rows[n_test+n_val:]

    os.makedirs(args.out_dir, exist_ok=True)
    for name, subset in [("train.csv", train_rows), ("val.csv", val_rows), ("test.csv", test_rows)]:
        outp = os.path.join(args.out_dir, name)
        with open(outp, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["file_path", "yaw"])
            for r in subset:
                w.writerow([r["file_path"], r["yaw"]])
        print(f"Wrote {name}: {len(subset)}")

if __name__ == "__main__":
    main()