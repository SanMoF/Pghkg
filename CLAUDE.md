Here's the `CLAUDE.md`:

```markdown
# CLAUDE.md — SCARA Robot Kinematics

## Project Context
C++ robotics library for a 4-DOF SCARA robot.
You have full read/write access to all files in this project.

## DH Parameter Convention
Column order is strictly: `[theta, d, alpha, r]`
- `theta` — joint angle (degrees)
- `d`     — link offset (prismatic joint displacement)
- `alpha` — link twist
- `r`     — link length

## Unit Convention
- **All angles in and out of every function must be in degrees**
- Use `DEG_TO_RAD` / `RAD_TO_DEG` macros for internal trig calls
- Never expose radians at the API boundary

## File Ownership
You own and must keep these files fully up to date after every change:
- `Robotics.h`
- `Robotics.cpp`

Always rewrite the full file — never show partial snippets.

---

## Required Structs (Robotics.h)

```cpp
struct IKSolution {
    float theta1; // degrees
    float theta2; // degrees
    float z;      // linear, same units as link lengths
    float theta4; // degrees
};
```

---

## Inverse Kinematics — Geometric Approach

Port the following MATLAB logic exactly. Do not substitute an algebraic
or Jacobian approach.

**MATLAB reference:**
```matlab
function [num_solutions, solutions] = InverseKinematics(op_var, DH)
    x = op_var(1); y = op_var(2); z = op_var(3); tool_angle = op_var(4);
    L1 = DH(1,4); L2 = DH(2,4);
    p = sqrt(x*x + y*y);
    solutions = zeros(2,4);
    if p > L1+L2 || p < abs(L1-L2)
        num_solutions = 0; return
    end
    gamma = atan2(y, x);
    alpha = acos((L1*L1 + p*p - L2*L2) / (2*L1*p));
    beta  = acos((L2*L2 + L1*L1 - p*p) / (2*L1*L2));
    solutions(1,1) = rad2deg(gamma - alpha);
    solutions(1,2) = rad2deg(pi - beta);
    solutions(1,3) = z;
    solutions(1,4) = tool_angle - solutions(1,1) - solutions(1,2);
    num_solutions = 1;
    if p == L1+L2 || p == abs(L1-L2), return, end
    solutions(2,1) = rad2deg(gamma + alpha);
    solutions(2,2) = rad2deg(beta - pi);
    solutions(2,3) = z;
    solutions(2,4) = tool_angle - solutions(2,1) - solutions(2,2);
    num_solutions = 2;
end
```

**C++ signature:**
```cpp
int Inverse_Kinematics(float DH_table[][4], float x, float y, float z,
                       float tool_angle_deg, IKSolution solutions[2]);
```

**Rules:**
- Extract `L1 = DH_table[0][3]`, `L2 = DH_table[1][3]` (r column, index 3)
- Input `tool_angle_deg` is in degrees
- All fields of `IKSolution` output in degrees
- Clamp `cos_alpha` and `cos_beta` to `[-1, 1]` before `acosf()`
- Singular check tolerance: `fabsf(p - (L1+L2)) < 1e-6f`

---

## Best Solution Selection

Port the following MATLAB logic exactly:

**MATLAB reference:**
```matlab
function index = findBestSolution(solutions, DH, weights)
    best_error = 1000000;
    index = 1;
    for i = 1:2
        error = 0;
        for j = 2:4
            error = error + weights(j-1) * abs(solutions(i,j) - DH(j,1));
            if error < best_error
                best_error = error;
                index = i;
            end
        end
    end
end
```

**C++ signature:**
```cpp
int findBestSolution(IKSolution solutions[2], int num_solutions,
                     float current_joints[4], float weights[3]);
```

**Rules:**
- `current_joints` order: `[theta1, theta2, z, theta4]` (degrees)
- Weights apply to `theta1` (w[0]), `theta2` (w[1]), `theta4` (w[2])
- Skip `z` — it is identical in both solutions
- Return index `0` or `1`
- If `num_solutions == 1`, return `0` immediately

---

## DH_to_HMatrix

Respect column order `[theta, d, alpha, r]`:
- `theta = DH_Parameters[0]` — convert to radians internally
- `d     = DH_Parameters[1]`
- `alpha = DH_Parameters[2]` — convert to radians internally
- `r     = DH_Parameters[3]`

Output is a 4×4 homogeneous matrix in row-major flat array (16 floats).

---

## General Rules
- Never break existing methods: `setup`, `setLinkLengths`, `Forward_Kinematics`,
  `Matrix_multiplication`
- All trig uses `cosf`, `sinf`, `atan2f`, `acosf`, `sqrtf`, `fabsf`
- Use `#define DEG_TO_RAD (M_PI / 180.0f)` and `#define RAD_TO_DEG (180.0f / M_PI)`
  defined at the top of `Robotics.cpp`
- No dynamic memory allocation
- No external dependencies beyond `<math.h>`
```
