# GitHub Upload Guide

Panduan langkah demi langkah untuk mempublikasikan project Robot Arena Navigation ke GitHub.

## Prerequisites

- Git installed di komputer Anda
- GitHub account (gratis di github.com)
- Project files sudah terorganisir

## Langkah 1: Install Git

### Windows
Download dan install dari: https://git-scm.com/download/win

### Linux
```bash
sudo apt-get install git
```

### Mac
```bash
brew install git
```

Verify instalasi:
```bash
git --version
```

## Langkah 2: Konfigurasi Git

```bash
git config --global user.name "Nama Anda"
git config --global user.email "email@anda.com"
```

## Langkah 3: Buat Repository di GitHub

1. Login ke github.com
2. Klik tombol **"+"** di kanan atas → **"New repository"**
3. Isi form:
   - **Repository name**: `robot-arena-navigation`
   - **Description**: `Multi-robot navigation system with computer vision and path planning`
   - **Public** atau **Private**: Pilih sesuai kebutuhan
   - **JANGAN** centang "Initialize with README" (kita sudah punya)
4. Klik **"Create repository"**

## Langkah 4: Organize Project Files

Struktur folder yang sudah dibuat:

```
robot-arena-navigation/
├── README.md                        ✓ Sudah dibuat
├── .gitignore                       ✓ Sudah dibuat
├── LICENSE                          ✓ Sudah dibuat
├── docs/
│   ├── setup-guide.md              ✓ Sudah dibuat
│   └── hardware-requirements.md    - Perlu dibuat (opsional)
├── vision-system/
│   ├── finalVision.py              ← Copy dari project Anda
│   ├── requirements.txt            ✓ Sudah dibuat
│   └── hsv_settings.yaml.example   ✓ Sudah dibuat
├── gui-application/
│   ├── MainForm.cs                 ← Copy dari project Anda
│   ├── DijkstraPathfinder.cs       ← Perlu dibuat (dari project Anda)
│   └── RobotArenaGUI.csproj        ← Copy dari project Anda
├── firmware/
│   ├── leader/
│   │   └── leader.ino              ← Rename dari leader.txt
│   └── follower/
│       └── follower.ino            ← Rename dari follower.txt
├── config/
│   └── network-settings.example.txt ✓ Sudah dibuat
└── examples/
    └── demo-scenarios/
```

## Langkah 5: Copy dan Organize Files

### A. Buat struktur folder

```bash
mkdir -p robot-arena-navigation/vision-system
mkdir -p robot-arena-navigation/gui-application
mkdir -p robot-arena-navigation/firmware/leader
mkdir -p robot-arena-navigation/firmware/follower
mkdir -p robot-arena-navigation/config
mkdir -p robot-arena-navigation/docs
mkdir -p robot-arena-navigation/examples/demo-scenarios
```

### B. Copy files ke struktur baru

**Vision System:**
```bash
# Copy finalVision.py ke folder baru
copy "finalVision - TESTING.py" robot-arena-navigation/vision-system/finalVision.py

# Files lain sudah dibuat oleh Claude
```

**GUI Application:**
```bash
# Copy semua file C# project Anda
copy MainForm.cs robot-arena-navigation/gui-application/
copy MainForm.Designer.cs robot-arena-navigation/gui-application/
copy Program.cs robot-arena-navigation/gui-application/
copy *.csproj robot-arena-navigation/gui-application/
# Jangan lupa file lainnya seperti DijkstraPathfinder.cs
```

**Firmware:**
```bash
# Rename dan copy firmware files
copy leader.txt robot-arena-navigation/firmware/leader/leader.ino
copy follower.txt robot-arena-navigation/firmware/follower/follower.ino
```

## Langkah 6: Initialize Git Repository

```bash
cd robot-arena-navigation
git init
```

## Langkah 7: Add Files to Git

```bash
# Add all files
git add .

# Verify apa saja yang akan dicommit
git status
```

Pastikan file-file berikut TIDAK muncul (sudah di-ignore):
- `bin/`, `obj/`, `.vs/` folders
- `__pycache__/`
- `hsv_settings.yaml` (hanya .example yang dicommit)
- `*.pyc`, `*.exe`, `*.dll`

## Langkah 8: Create First Commit

```bash
git commit -m "Initial commit: Multi-robot navigation system

- Computer vision tracking with ArUco markers
- GUI application for mission planning
- Leader-follower robot coordination
- Dijkstra pathfinding with obstacle avoidance
- Complete documentation and setup guides"
```

## Langkah 9: Connect to GitHub

GitHub akan memberikan commands setelah membuat repository. Jalankan:

```bash
# Add remote repository
git remote add origin https://github.com/username/robot-arena-navigation.git

# Verify remote
git remote -v
```

Ganti `username` dengan username GitHub Anda.

## Langkah 10: Push to GitHub

```bash
# Push ke branch main
git branch -M main
git push -u origin main
```

Anda akan diminta login:
- Username: GitHub username Anda
- Password: **Personal Access Token** (bukan password biasa)

### Membuat Personal Access Token

1. GitHub → Settings → Developer settings → Personal access tokens → Tokens (classic)
2. Generate new token → Pilih expiration (30 days, 60 days, dll)
3. Pilih scopes: minimal `repo` (full control of private repositories)
4. Generate token → **COPY dan simpan** (tidak akan ditampilkan lagi!)
5. Gunakan token ini sebagai password saat push

## Langkah 11: Verify Upload

1. Buka `https://github.com/username/robot-arena-navigation`
2. Pastikan semua files terlihat
3. README.md akan otomatis tampil di homepage
4. Check commits, branches, files

## Langkah 12: Add Additional Files (Opsional)

### A. Screenshots/Images

Jika ada screenshots atau diagram:

```bash
mkdir docs/images
# Copy gambar ke folder ini
copy screenshot1.png docs/images/
git add docs/images/
git commit -m "docs: Add screenshots and diagrams"
git push
```

### B. GitHub Issues Template

Buat `.github/ISSUE_TEMPLATE/bug_report.md`:
```markdown
---
name: Bug Report
about: Create a report to help us improve
title: '[BUG] '
labels: bug
assignees: ''
---

**Describe the bug**
A clear description of what the bug is.

**To Reproduce**
Steps to reproduce the behavior

**Expected behavior**
What you expected to happen

**Screenshots**
If applicable, add screenshots

**Environment:**
 - OS: [e.g. Windows 10]
 - Vision System Python Version: [e.g. 3.9]
 - GUI .NET Version: [e.g. 4.7.2]
 - Hardware: [e.g. ESP8266 NodeMCU]

**Additional context**
Add any other context about the problem here
```

```bash
git add .github/
git commit -m "chore: Add issue templates"
git push
```

## Langkah 13: Setup GitHub Pages (Opsional)

Untuk hosting dokumentasi:

1. GitHub repository → Settings → Pages
2. Source: Deploy from a branch
3. Branch: `main` / `docs`
4. Save

Dokumentasi akan tersedia di:
`https://username.github.io/robot-arena-navigation/`

## Langkah 14: Add Topics/Tags

Di halaman repository GitHub:
1. Klik gear icon di sebelah "About"
2. Add topics:
   - `robotics`
   - `computer-vision`
   - `path-planning`
   - `esp8266`
   - `opencv`
   - `csharp`
   - `python`
   - `arduino`
   - `autonomous-navigation`
   - `leader-follower`

## Update README dengan Badges

Edit README.md untuk menambahkan badges di bagian atas:

```markdown
# Robot Arena Navigation System

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Python](https://img.shields.io/badge/python-3.8%2B-blue)
![.NET](https://img.shields.io/badge/.NET-4.7.2%2B-purple)
![Platform](https://img.shields.io/badge/platform-ESP8266-green)

A comprehensive multi-robot navigation system...
```

Commit dan push:
```bash
git add README.md
git commit -m "docs: Add status badges"
git push
```

## Maintenance: Update Repository

Saat ada perubahan di project:

```bash
# Check status
git status

# Add perubahan
git add .

# Commit dengan pesan yang jelas
git commit -m "feat: Add dynamic obstacle avoidance"

# Push ke GitHub
git push
```

### Commit Message Conventions

Gunakan conventional commits:
- `feat:` - New feature
- `fix:` - Bug fix
- `docs:` - Documentation changes
- `refactor:` - Code refactoring
- `test:` - Adding tests
- `chore:` - Build process, dependencies

Examples:
```bash
git commit -m "feat: Add A* pathfinding algorithm"
git commit -m "fix: Resolve encoder direction issue"
git commit -m "docs: Update hardware requirements"
git commit -m "refactor: Optimize PID control loop"
```

## Create Releases

Saat project mencapai milestone:

1. GitHub → Releases → Create a new release
2. Tag: `v1.0.0` (semantic versioning)
3. Title: "Initial Release"
4. Description: Changelog dan fitur utama
5. Attach binaries jika ada (compiled GUI, firmware hex files)
6. Publish release

## Troubleshooting

### Problem: "Permission denied"

**Solution:**
```bash
# Use HTTPS instead of SSH
git remote set-url origin https://github.com/username/robot-arena-navigation.git
```

### Problem: "Repository not found"

**Solution:**
- Verify repository name di GitHub
- Check spelling dan case-sensitivity
- Pastikan sudah login dengan account yang benar

### Problem: "Failed to push"

**Solution:**
```bash
# Pull latest changes first
git pull origin main --allow-unrelated-histories

# Resolve conflicts if any
# Then push again
git push origin main
```

### Problem: Accidentally committed large files

**Solution:**
```bash
# Remove from git but keep locally
git rm --cached large-file.bin

# Add to .gitignore
echo "large-file.bin" >> .gitignore

# Commit
git add .gitignore
git commit -m "chore: Remove large file and update gitignore"
git push
```

### Problem: Want to undo last commit

```bash
# Undo commit but keep changes
git reset --soft HEAD~1

# Undo commit and discard changes (CAREFUL!)
git reset --hard HEAD~1
```

## Security Best Practices

### Never Commit:
- WiFi passwords (use example config files)
- API keys or tokens
- Personal IP addresses (use placeholders)
- Binary executables (unless as release assets)

### Check Before Committing:
```bash
# Review what will be committed
git diff

# Check specific file
git diff filename.cs
```

### Remove Sensitive Data if Accidentally Committed:
```bash
# Remove from history (CAREFUL - rewrites history)
git filter-branch --force --index-filter \
  "git rm --cached --ignore-unmatch path/to/sensitive/file" \
  --prune-empty --tag-name-filter cat -- --all

# Force push (only if repository is not shared yet)
git push origin --force --all
```

## Next Steps After Upload

1. **Star Your Own Repo** (untuk testing)
2. **Share Link** dengan collaborators
3. **Create Projects** (GitHub Projects untuk tracking)
4. **Setup CI/CD** (GitHub Actions untuk automated testing - opsional)
5. **Write Wiki** (untuk dokumentasi lebih lengkap)
6. **Engage Community** (respond to issues, PRs)

## Useful Git Commands

```bash
# View commit history
git log --oneline

# View changes
git diff

# Discard local changes
git checkout -- filename

# Create new branch
git checkout -b feature-name

# Switch branch
git checkout main

# Merge branch
git merge feature-name

# Delete branch
git branch -d feature-name

# View remote info
git remote show origin

# Fetch latest from remote
git fetch origin

# Pull latest changes
git pull origin main
```

---

## Resources

- [Git Documentation](https://git-scm.com/doc)
- [GitHub Guides](https://guides.github.com/)
- [Markdown Guide](https://www.markdownguide.org/)
- [Semantic Versioning](https://semver.org/)
- [Conventional Commits](https://www.conventionalcommits.org/)

---

**Selamat!** Project Anda sekarang sudah live di GitHub! 🎉
