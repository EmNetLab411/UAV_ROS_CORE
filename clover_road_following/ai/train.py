#!/usr/bin/env python3
import os
import csv
import math
import random
import argparse
from typing import List, Tuple

import numpy as np
from PIL import Image, ImageFilter
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader, random_split
from torch.utils.tensorboard import SummaryWriter
import torchvision.transforms.functional as TF

# -------------------------
# Utils: Pillow resample compat
# -------------------------
def _resample_bilinear():
    # Pillow >= 9: Image.Resampling.BILINEAR, cũ: Image.BILINEAR
    try:
        return Image.Resampling.BILINEAR  # type: ignore[attr-defined]
    except Exception:
        return Image.BILINEAR

# -------------------------
# Model: Small PilotNet
# -------------------------
class SmallPilotNet(nn.Module):
    def __init__(self, input_h=90, input_w=160):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 24, kernel_size=5, stride=2), nn.ELU(inplace=True),
            nn.Conv2d(24, 36, kernel_size=5, stride=2), nn.ELU(inplace=True),
            nn.Conv2d(36, 48, kernel_size=5, stride=2), nn.ELU(inplace=True),
            nn.Conv2d(48, 64, kernel_size=3, stride=1), nn.ELU(inplace=True),
            nn.Conv2d(64, 64, kernel_size=3, stride=1), nn.ELU(inplace=True),
        )
        # compute flatten size
        with torch.no_grad():
            dummy = torch.zeros(1, 3, input_h, input_w)
            n_flat = self.features(dummy).view(1, -1).shape[1]
        self.classifier = nn.Sequential(
            nn.Dropout(0.2),
            nn.Linear(n_flat, 100), nn.ELU(inplace=True),
            nn.Linear(100, 50), nn.ELU(inplace=True),
            nn.Linear(50, 10), nn.ELU(inplace=True),
            nn.Linear(10, 1)
        )

    def forward(self, x):
        x = self.features(x)
        x = torch.flatten(x, 1)
        x = self.classifier(x)
        return x.squeeze(1)

# -------------------------
# Dataset + Augmentations
# -------------------------
class RoadDataset(Dataset):
    def __init__(self, csv_path: str, root_dir: str, img_size=(160, 90),
                 is_train=True, flip_prob=0.5, jitter=0.2, noise_std=0.02, motion_blur_prob=0.2, seed=42):
        self.root_dir = root_dir
        self.img_w, self.img_h = img_size
        self.is_train = is_train
        self.flip_prob = flip_prob
        self.jitter = jitter
        self.noise_std = noise_std
        self.motion_blur_prob = motion_blur_prob
        self.rng = random.Random(seed)

        self.samples = []
        with open(os.path.join(root_dir, csv_path), "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                path = row["file_path"]
                yaw = float(row["yaw"])
                self.samples.append((path, yaw))

        self._resample = _resample_bilinear()

    def __len__(self):
        return len(self.samples)

    def apply_motion_blur(self, pil_img: Image.Image, k=None, direction=None) -> Image.Image:
        """
        Motion blur an toàn:
        - k: số lẻ >=3; nếu None -> chọn trong [3,5,7,9]
        - direction: 'h' (ngang), 'v' (dọc), 'diag', 'anti'; nếu None -> random
        """
        try:
            if k is None:
                k = self.rng.choice([3, 5, 7, 9])
            k = int(k)
            if k < 3:
                k = 3
            if k % 2 == 0:
                k += 1  # đảm bảo lẻ

            # Tạo kernel k×k
            kernel = np.zeros((k, k), dtype=np.float32)

            if direction is None:
                direction = self.rng.choice(['h', 'v', 'diag', 'anti'])

            if direction == 'h':
                kernel[k // 2, :] = 1.0
            elif direction == 'v':
                kernel[:, k // 2] = 1.0
            elif direction == 'diag':
                np.fill_diagonal(kernel, 1.0)
            else:  # 'anti'
                kernel = np.fliplr(np.eye(k, dtype=np.float32))

            s = float(kernel.sum())
            if s <= 0:
                # fallback an toàn
                kernel[:] = 0.0
                kernel[k // 2, :] = 1.0
                s = float(kernel.sum())
            kernel /= s

            flat = kernel.flatten().tolist()
            return pil_img.filter(ImageFilter.Kernel((k, k), flat, scale=1.0, offset=0))
        except Exception:
            # fallback: Gaussian blur nhẹ, tránh văng lỗi DataLoader
            try:
                radius = self.rng.uniform(0.5, 1.5)
                return pil_img.filter(ImageFilter.GaussianBlur(radius=radius))
            except Exception:
                return pil_img

    def __getitem__(self, idx):
        rel_path, yaw = self.samples[idx]
        img_path = os.path.join(self.root_dir, rel_path)

        # Đọc ảnh an toàn
        with Image.open(img_path) as im:
            img = im.convert("RGB")

        # Resize to (W,H)
        img = img.resize((self.img_w, self.img_h), resample=self._resample)

        if self.is_train:
            # Motion blur (trước khi jitter để phân bố hợp lý)
            if self.rng.random() < self.motion_blur_prob:
                img = self.apply_motion_blur(img)

            # Color jitter
            if self.jitter > 1e-6:
                b = 1.0 + self.rng.uniform(-self.jitter, self.jitter)
                c = 1.0 + self.rng.uniform(-self.jitter, self.jitter)
                img = TF.adjust_brightness(img, max(0.0, b))
                img = TF.adjust_contrast(img, max(0.0, c))

            # Horizontal flip (invert yaw)
            if self.rng.random() < self.flip_prob:
                img = TF.hflip(img)
                yaw = -yaw

        # To tensor [0,1]
        img_t = TF.to_tensor(img)

        # Add Gaussian noise
        if self.is_train and self.noise_std > 1e-6:
            noise = torch.randn_like(img_t) * self.noise_std
            img_t = torch.clamp(img_t + noise, 0.0, 1.0)

        return img_t, torch.tensor(yaw, dtype=torch.float32)

# -------------------------
# Utils
# -------------------------
def set_seed(seed=42):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)

def save_onnx(model, onnx_path, input_h=90, input_w=160, device='cpu'):
    model.eval()
    dummy = torch.zeros(1, 3, input_h, input_w).to(device)
    torch.onnx.export(
        model, dummy, onnx_path,
        input_names=['input'], output_names=['yaw'],
        dynamic_axes={'input': {0: 'batch'}, 'yaw': {0: 'batch'}},
        opset_version=12, do_constant_folding=True
    )

# -------------------------
# Train
# -------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data_root", type=str, required=True, help="root dir containing images/ and csvs")
    ap.add_argument("--train_csv", type=str, default="train.csv")
    ap.add_argument("--val_csv", type=str, default="val.csv")
    ap.add_argument("--test_csv", type=str, default="test.csv")
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--batch_size", type=int, default=64)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--weight_decay", type=float, default=1e-5)
    ap.add_argument("--early_patience", type=int, default=8)
    ap.add_argument("--log_dir", type=str, default="runs/exp1")
    ap.add_argument("--out_dir", type=str, default="outputs/exp1")
    ap.add_argument("--img_w", type=int, default=160)
    ap.add_argument("--img_h", type=int, default=90)
    ap.add_argument("--flip_prob", type=float, default=0.5)
    ap.add_argument("--jitter", type=float, default=0.2)
    ap.add_argument("--noise_std", type=float, default=0.02)
    ap.add_argument("--motion_blur_prob", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    writer = SummaryWriter(args.log_dir)
    set_seed(args.seed)

    device = 'cuda' if torch.cuda.is_available() else 'cpu'

    train_ds = RoadDataset(args.train_csv, args.data_root, (args.img_w, args.img_h), is_train=True,
                           flip_prob=args.flip_prob, jitter=args.jitter, noise_std=args.noise_std,
                           motion_blur_prob=args.motion_blur_prob, seed=args.seed)
    val_ds = RoadDataset(args.val_csv, args.data_root, (args.img_w, args.img_h), is_train=False, seed=args.seed)
    test_ds = RoadDataset(args.test_csv, args.data_root, (args.img_w, args.img_h), is_train=False, seed=args.seed)

    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True, num_workers=4, pin_memory=True)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False, num_workers=4, pin_memory=True)
    test_loader = DataLoader(test_ds, batch_size=args.batch_size, shuffle=False, num_workers=4, pin_memory=True)

    model = SmallPilotNet(args.img_h, args.img_w).to(device)
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)

    best_val = float('inf')
    best_epoch = -1
    patience = args.early_patience
    ckpt_path = os.path.join(args.out_dir, "model_best.pt")

    global_step = 0
    for epoch in range(1, args.epochs + 1):
        model.train()
        train_loss = 0.0
        for imgs, yaws in train_loader:
            imgs, yaws = imgs.to(device), yaws.to(device)
            preds = model(imgs)
            loss = criterion(preds, yaws)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            train_loss += loss.item() * imgs.size(0)
            writer.add_scalar("train/loss", loss.item(), global_step)
            global_step += 1
        train_loss /= len(train_ds)

        # Validation
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for imgs, yaws in val_loader:
                imgs, yaws = imgs.to(device), yaws.to(device)
                preds = model(imgs)
                loss = criterion(preds, yaws)
                val_loss += loss.item() * imgs.size(0)
        val_loss /= len(val_ds)
        writer.add_scalar("val/loss", val_loss, epoch)

        print(f"Epoch {epoch:03d}: train_loss={train_loss:.6f}, val_loss={val_loss:.6f}")

        if val_loss < best_val - 1e-6:
            best_val = val_loss
            best_epoch = epoch
            torch.save({"model": model.state_dict(), "epoch": epoch}, ckpt_path)
            print(f"  Saved new best: val_loss={best_val:.6f} at epoch {epoch}")
        elif epoch - best_epoch >= patience:
            print(f"Early stopping at epoch {epoch} (best at {best_epoch})")
            break

    # Load best and test
    ckpt = torch.load(ckpt_path, map_location=device)
    model.load_state_dict(ckpt["model"])
    model.eval()
    test_loss = 0.0
    with torch.no_grad():
        for imgs, yaws in test_loader:
            imgs, yaws = imgs.to(device), yaws.to(device)
            preds = model(imgs)
            loss = criterion(preds, yaws)
            test_loss += loss.item() * imgs.size(0)
    test_loss /= len(test_ds)
    writer.add_scalar("test/loss", test_loss)
    print(f"Test MSE: {test_loss:.6f}")

    # Export ONNX
    onnx_path = os.path.join(args.out_dir, "model_best.onnx")
    save_onnx(model.to('cpu'), onnx_path, input_h=args.img_h, input_w=args.img_w, device='cpu')
    print(f"Exported ONNX: {onnx_path}")

    writer.close()

if __name__ == "__main__":
    main()