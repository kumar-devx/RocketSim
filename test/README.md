# 🧪 **RocketSim GPU-ONLY Tests**

## **Available Tests:**

### **SimpleCudaTest.exe** ✅ **READY**
Tests basic CUDA functionality without RocketSim dependencies.

**What it tests:**
- ✅ CUDA device detection
- ✅ GPU memory allocation (1KB to 100MB)
- ✅ Host ↔ Device memory transfers
- ✅ Data integrity verification

---

## **How to Run Tests in VS Code:**

### **Method 1: Using Tasks (Recommended)**

1. Press `Ctrl+Shift+P`
2. Type: `Tasks: Run Task`
3. Select: **"Run SimpleCudaTest"**

### **Method 2: Using Debug/Run**

1. Press `F5` or click "Run and Debug" in sidebar
2. Select: **"Run SimpleCudaTest"**
3. View output in integrated terminal

### **Method 3: Command Line**

```powershell
# From project root
.\build\SimpleCudaTest.exe
```

---

## **Expected Output:**

### ✅ **If GPU Working:**
```
╔══════════════════════════════════════╗
║  Simple CUDA Functionality Test      ║
╚══════════════════════════════════════╝

TEST 1: CUDA Device Detection
======================================
Device Count: 1
Status: no error
✅ Found 1 CUDA device(s)

TEST 2: Device Properties
======================================
Device 0: NVIDIA GeForce RTX 5070
  Compute Capability: 8.9
  Total Memory: 12226 MB
  ...

🎉 ALL TESTS PASSED!
```

### ❌ **If GPU Not Working:**
```
TEST 1: CUDA Device Detection
======================================
Device Count: 0
Status: CUDA driver version is insufficient
❌ No CUDA devices!
```

---

## **Building Tests:**

### **Automatic (via VS Code):**
Tests are automatically built when you run them (F5)

### **Manual (via CMake):**
```powershell
cmake -B build -G Ninja -DROCKETSIM_CUDA=ON -DBUILD_CUDA_TEST=ON
cmake --build build --target SimpleCudaTest
```

---

## **Configuration:**

### **GPU-ONLY Mode:** ✅ **ACTIVE**
- CPU fallback: **DISABLED**
- CUDA required: **YES**
- Will crash if no GPU detected

### **Supported GPUs:**
- RTX 2000+ (Turing)
- RTX 3000+ (Ampere)
- RTX 4000+ (Ada Lovelace)
- RTX 5000+ (Blackwell)
- A100, H100 (Data center)

### **Required Drivers:**
- CUDA 12.6+
- Driver 545.x+ (for CUDA 12.6)
- Driver 571.x+ (for RTX 5070)

---

## **Troubleshooting:**

### **Error: "No CUDA devices found"**
1. Update GPU drivers
2. Check GPU is visible: `nvidia-smi`
3. Verify CUDA installed: `nvcc --version`

### **Error: "Driver version insufficient"**
1. Download latest drivers from nvidia.com
2. Install Studio Drivers (not Game Ready)
3. Reboot

### **Test won't run in VS Code**
1. Reload VS Code window (`Ctrl+Shift+P` → "Reload Window")
2. Check `.vscode/tasks.json` exists
3. Try running from PowerShell: `.\build\SimpleCudaTest.exe`

---

## **VS Code Keyboard Shortcuts:**

| Action | Shortcut |
|--------|----------|
| Run test | `F5` |
| Build library | `Ctrl+Shift+B` |
| Run task | `Ctrl+Shift+P` → Tasks |
| Open terminal | `` Ctrl+` `` |
| Clean build | `Ctrl+Shift+P` → CMake: Clean |

---

## **What to Do After Tests Pass:**

1. ✅ Tests passed? **GPU is working!**
2. Copy library to your project:
   ```powershell
   Copy-Item "build\RocketSim.lib" "D:\project\GigaLearnCPP-Leak\out\build\x64-Release\"
   ```
3. Rebuild your GigaLearn project
4. Deploy to GPU server

---

**Your GPU-ONLY RocketSim is ready! 🚀**
