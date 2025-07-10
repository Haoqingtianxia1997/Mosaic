#!/usr/bin/env python3
import subprocess
import re
import shlex

def call_fk_service():
    """
    通过 shell 调用 `ros2 service call /fk_service action_interfaces/srv/Fk "{}"`
    返回 (x, y, z) 浮点数元组
    """
    cmd = 'ros2 service call /fk_service action_interfaces/srv/Fk "{}"'
    # 用 shlex.split 避免引号问题
    result = subprocess.run(
        shlex.split(cmd),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    if result.returncode != 0:
        raise RuntimeError(f"ros2 命令失败: {result.stderr}")

    # 在输出里找 x=..., y=..., z=...
    # 兼容下面两种格式：
    #   action_interfaces.srv.Fk_Response(x=0.39, y=0.29, z=0.30)
    #   x: 0.39
    pattern = r'[xyz]\s*=\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)'
    matches = re.findall(pattern, result.stdout)
    if len(matches) >= 3:
        x, y, z = map(float, matches[:3])
        return x, y, z
    else:
        # 第二种 YAML 风格
        yaml_pat = r'^\s*[xyz]:\s*([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)\s*$'
        found = {}
        for line in result.stdout.splitlines():
            m = re.match(yaml_pat, line)
            if m:
                key = line.strip().split(':')[0]
                found[key] = float(m.group(1))
        if {'x', 'y', 'z'} <= found.keys():
            return found['x'], found['y'], found['z']
        raise ValueError("未能从输出中解析到 x y z")

if __name__ == '__main__':
    try:
        x, y, z = call_fk_service()
        print(f"末端位置 -> x: {x:.4f}, y: {y:.4f}, z: {z:.4f}")
    except Exception as e:
        print(f"调用失败: {e}")
