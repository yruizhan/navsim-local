# ESDF 算法一致性对比分析

## 🎯 目的

对比原始参考实现（`sdf_map.cpp`）和新实现（`esdf_map.cpp`）的 ESDF 计算逻辑，确保算法完全一致。

---

## 📊 核心差异总结

### ❌ 发现的关键差异

| 项目 | 原始实现 (SDFmap) | 新实现 (ESDFMap) | 影响 |
|------|------------------|-----------------|------|
| **循环边界** | `q <= end` | `q < end` | ⚠️ **严重** |
| **数组索引** | `k = start` | `k = 0` | ⚠️ **严重** |
| **开平方位置** | 在 lambda 中 | 在循环后 | ⚠️ **严重** |
| **边界检查** | `s <= z[k]` | `s <= z[k] && k > 0` | ⚠️ **中等** |

---

## 🔍 详细对比

### 1. fillESDF() 函数签名

#### 原始实现 (sdf_map.cpp:661)
```cpp
template <typename F_get_val, typename F_set_val>
void SDFmap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim_size)
```

#### 新实现 (esdf_map.cpp:121)
```cpp
template <typename F_get_val, typename F_set_val>
void ESDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim_size)
```

✅ **一致**

---

### 2. 数组初始化

#### 原始实现 (sdf_map.cpp:662-668)
```cpp
int v[dim_size];              // ❌ VLA（可变长度数组）
double z[dim_size + 1];

int k = start;                // ⚠️ k 初始化为 start
v[start] = start;
z[start] = -std::numeric_limits<double>::max();
z[start + 1] = std::numeric_limits<double>::max();
```

#### 新实现 (esdf_map.cpp:126-132)
```cpp
std::vector<int> v(dim_size);        // ✅ 使用 std::vector
std::vector<double> z(dim_size + 1);

int k = 0;                           // ⚠️ k 初始化为 0
v[0] = start;
z[0] = -std::numeric_limits<double>::max();
z[1] = std::numeric_limits<double>::max();
```

⚠️ **差异**：
- VLA vs std::vector：**无影响**（只是内存分配方式不同）
- `k = start` vs `k = 0`：**有影响**（索引方式不同）

---

### 3. 第一个循环（构建包络）

#### 原始实现 (sdf_map.cpp:670-684)
```cpp
for (int q = start + 1; q <= end; q++) {  // ⚠️ q <= end（包含 end）
  k++;
  double s;
  
  do {
    k--;
    s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
  } while (s <= z[k]);  // ⚠️ 没有 k > 0 检查
  
  k++;
  v[k] = q;
  z[k] = s;
  z[k + 1] = std::numeric_limits<double>::max();
}
```

#### 新实现 (esdf_map.cpp:134-149)
```cpp
for (int q = start + 1; q < end; q++) {  // ⚠️ q < end（不包含 end）
  k++;
  double s;
  do {
    k--;
    double val_q = f_get_val(q);
    double val_vk = f_get_val(v[k]);
    s = ((val_q + static_cast<double>(q * q)) - (val_vk + static_cast<double>(v[k] * v[k]))) /
        (2.0 * static_cast<double>(q - v[k]));
  } while (s <= z[k] && k > 0);  // ⚠️ 添加了 k > 0 检查
  
  k++;
  v[k] = q;
  z[k] = s;
  z[k + 1] = std::numeric_limits<double>::max();
}
```

⚠️ **严重差异**：
1. **循环边界**：`q <= end` vs `q < end`
2. **边界检查**：`s <= z[k]` vs `s <= z[k] && k > 0`

---

### 4. 第二个循环（计算距离）

#### 原始实现 (sdf_map.cpp:686-692)
```cpp
k = start;  // ⚠️ k 重置为 start

for (int q = start; q <= end; q++) {  // ⚠️ q <= end
  while (z[k + 1] < q) k++;
  double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
  f_set_val(q, val);
}
// ⚠️ 没有开平方
```

#### 新实现 (esdf_map.cpp:151-164)
```cpp
k = 0;  // ⚠️ k 重置为 0

for (int q = start; q < end; q++) {  // ⚠️ q < end
  while (z[k + 1] < static_cast<double>(q)) {
    k++;
  }
  double val_vk = f_get_val(v[k]);
  double dx = static_cast<double>(q - v[k]);
  f_set_val(q, val_vk + dx * dx);
}

// ⚠️ 添加了开平方
for (int q = start; q < end; q++) {
  f_set_val(q, std::sqrt(f_get_val(q)));
}
```

⚠️ **严重差异**：
1. **循环边界**：`q <= end` vs `q < end`
2. **开平方位置**：原始实现在调用处开平方，新实现在函数内开平方

---

### 5. 调用方式对比

#### 原始实现 (sdf_map.cpp:611-618)
```cpp
for (int x = 0; x <= update_X_SIZE; x++) {
  fillESDF(
    [&](int y) {
      return gridmap_[...] == Occupied ? 0.0 : std::numeric_limits<double>::max();
    },
    [&](int y, double val) { tmp_buffer1_[x * update_Y_SIZE + y] = val; },
    0,                    // start = 0
    update_Y_SIZE,        // end = update_Y_SIZE
    update_Y_SIZE+1       // dim_size = update_Y_SIZE+1
  );
}

for (int y = 0; y <= update_Y_SIZE; y++) {
  fillESDF(
    [&](int x) { return tmp_buffer1_[x * update_Y_SIZE + y]; },
    [&](int x, double val) {
      distance_buffer_[x * update_Y_SIZE + y] = grid_interval_ * std::sqrt(val);  // ⚠️ 在这里开平方
    },
    0, update_X_SIZE, update_X_SIZE+1
  );
}
```

#### 新实现 (esdf_map.cpp:83-84)
```cpp
for (int y = 0; y < GLY_SIZE_; ++y) {
  fillESDF(f_get_val_x, f_set_val_x,
    y * GLX_SIZE_,        // start = y * GLX_SIZE_
    (y + 1) * GLX_SIZE_,  // end = (y + 1) * GLX_SIZE_
    GLX_SIZE_             // dim_size = GLX_SIZE_
  );
}
```

⚠️ **严重差异**：
1. **start 参数**：原始实现总是 `start = 0`，新实现 `start = y * GLX_SIZE_`
2. **开平方位置**：原始实现在 lambda 中开平方，新实现在 `fillESDF()` 内部开平方

---

## 🚨 问题分析

### 问题 1：循环边界不一致

**原始**：`for (int q = start + 1; q <= end; q++)`  
**新实现**：`for (int q = start + 1; q < end; q++)`

**影响**：
- 原始实现处理 `[start+1, end]`（包含 end）
- 新实现处理 `[start+1, end)`（不包含 end）
- **结果**：新实现会少处理一个元素

### 问题 2：数组索引不一致

**原始**：`k = start`, `v[start] = start`, `z[start] = ...`  
**新实现**：`k = 0`, `v[0] = start`, `z[0] = ...`

**影响**：
- 原始实现使用绝对索引（`v[start]`, `z[start]`）
- 新实现使用相对索引（`v[0]`, `z[0]`）
- **结果**：当 `start != 0` 时，数组访问模式完全不同

### 问题 3：开平方位置不一致

**原始**：在调用处的 lambda 中开平方  
**新实现**：在 `fillESDF()` 函数内部开平方

**影响**：
- 原始实现：`fillESDF()` 输出平方距离，调用者负责开平方
- 新实现：`fillESDF()` 输出欧氏距离
- **结果**：如果调用者期望平方距离，会得到错误结果

---

## ✅ 修复方案

### 方案：完全对齐原始实现

需要修改 `esdf_map.cpp` 的 `fillESDF()` 函数，使其与原始实现**完全一致**：

```cpp
template <typename F_get_val, typename F_set_val>
void ESDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim_size) {
  // 使用 vector 代替 VLA（这是唯一允许的改动）
  std::vector<int> v(dim_size);
  std::vector<double> z(dim_size + 1);

  int k = start;  // ✅ 改回 start
  v[start] = start;  // ✅ 使用绝对索引
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {  // ✅ 改回 q <= end
    k++;
    double s;
    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);  // ✅ 移除 k > 0 检查

    k++;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;  // ✅ 改回 start
  for (int q = start; q <= end; q++) {  // ✅ 改回 q <= end
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
  // ✅ 移除开平方（由调用者负责）
}
```

### 同时修改 computeESDF()

```cpp
void ESDFMap::computeESDF() {
  // ... 初始化代码保持不变 ...

  // X 方向扫描
  for (int y = 0; y < GLY_SIZE_; ++y) {
    fillESDF(f_get_val_x, f_set_val_x,
      y * GLX_SIZE_,        // start
      y * GLX_SIZE_ + GLX_SIZE_ - 1,  // ✅ end = start + GLX_SIZE_ - 1
      GLX_SIZE_             // dim_size
    );
  }

  // Y 方向扫描
  std::vector<double> temp_buffer(GLY_SIZE_);
  
  for (int x = 0; x < GLX_SIZE_; ++x) {
    // 提取列数据
    for (int y = 0; y < GLY_SIZE_; ++y) {
      temp_buffer[y] = distance_buffer_all_[Index2Vectornum(x, y)];
    }

    // 对列进行距离变换
    auto f_get_val_y = [&](const int idx) -> double& {
      return temp_buffer[idx];
    };
    auto f_set_val_y = [&](const int idx, const double val) {
      temp_buffer[idx] = val;
    };

    fillESDF(f_get_val_y, f_set_val_y,
      0,                // start = 0
      GLY_SIZE_ - 1,    // ✅ end = GLY_SIZE_ - 1
      GLY_SIZE_         // dim_size
    );

    // 写回列数据（开平方）
    for (int y = 0; y < GLY_SIZE_; ++y) {
      distance_buffer_all_[Index2Vectornum(x, y)] = std::sqrt(temp_buffer[y]);  // ✅ 在这里开平方
    }
  }

  // ... 后续代码保持不变 ...
}
```

---

## 📋 修复检查清单

- [ ] `fillESDF()` 循环边界改为 `q <= end`
- [ ] `fillESDF()` 数组索引改为 `k = start`, `v[start]`, `z[start]`
- [ ] `fillESDF()` 移除 `k > 0` 边界检查
- [ ] `fillESDF()` 移除内部开平方
- [ ] `computeESDF()` 调用时 `end` 参数改为 `start + size - 1`
- [ ] `computeESDF()` 在调用后开平方
- [ ] 测试 ESDF 计算结果是否正确
- [ ] 测试可视化是否正常

---

## ⚠️ 重要提醒

1. **不要为了修复可视化而修改核心算法**
2. **可视化问题应该在数据适配层解决**（已在 `esdf_builder_plugin.cpp` 中取绝对值）
3. **JPS 规划器依赖正确的 ESDF 计算**，任何偏差都可能导致路径规划失败

---

## 🎯 下一步

1. 修复 `esdf_map.cpp` 中的 `fillESDF()` 和 `computeESDF()` 函数
2. 重新编译测试
3. 验证 ESDF 计算结果是否与原始实现一致
4. 确认可视化正常（通过数据适配层的绝对值转换）

