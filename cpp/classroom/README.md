# 课堂演示服务

供老师上课演示：在和机器人同一网络的 Ubuntu 电脑上运行一个小服务，把 C++ 程序发给它，服务用 `hanppie.h` 编译并运行，实时返回程序输出，可以随时停止。编译选项与 Dev-C++ 一致（`-O2 -std=c++11`），同一份代码两边都能用。

只服务一台机器人、一次一个程序；有程序在运行时再提交会被拒绝，不排队。只用 Python 标准库。

## 安装（Ubuntu）

需要 Python 3.10+ 和 `g++`：

```bash
sudo apt install python3 g++
sudo mkdir -p /opt/hanppie-classroom
sudo cp cpp/classroom/runner.py /opt/hanppie-classroom/
# hanppie.h 从 Release 附件下载，或在仓库里生成：
sudo python3 cpp/tools/amalgamate.py /opt/hanppie-classroom/hanppie.h
```

开机自动启动：

```bash
sudo cp cpp/classroom/hanppie-classroom.service /etc/systemd/system/
sudo systemctl enable --now hanppie-classroom
```

服务以临时的普通用户运行：系统目录只读，不能访问 home 目录，内存上限 768 MB，进程和线程总数上限 64。也可以不装成服务，直接运行 `python3 cpp/classroom/runner.py --header <hanppie.h 路径>`，这时没有上述限制。

## 使用

```bash
curl -N --data-binary @demo.cpp http://127.0.0.1:8000/run   # 编译并运行，实时显示输出
curl -X POST http://127.0.0.1:8000/stop                     # 停止
curl http://127.0.0.1:8000/status                           # {"state": "idle"} 或 "running"
```

| 接口 | 说明 |
| --- | --- |
| `POST /run` | 请求内容是 C++ 源码（不超过 256 KB）。编译失败返回 422 和编译器输出；成功则返回 200，实时输出程序内容，最后一行是 `[runner] ...` 说明结束原因。已有程序在运行时返回 409 |
| `POST /stop` | 停止当前程序，返回 `{"stopped": true}`；没有程序在运行时为 `false`。在 `curl` 里按 Ctrl+C 断开 `/run` 也会停止 |
| `GET /status` | 空闲还是正在运行 |

- **停止方式**：先发 SIGTERM，SDK 收到后先停车再退出；2 秒内没退出就强制结束。单次运行最长 3 分钟（`--run-seconds` 可改），输出最多 1 MB，超出都会停止程序。
- **机器人**：程序里用入门接口的 `connect()` 或 `RobotCtrl rm;` 自动发现机器人，不需要配置 IP 和 AppID。官方 App 要先关掉。
- **访问范围**：默认只监听本机（`--host 127.0.0.1`）。用 `--host 0.0.0.0` 允许教室里其他电脑访问时，同一网络里的任何人都能在这台电脑上运行代码，只适合可信的教室网络。

已用模拟机器人验证编译、运行、停止（停止后机器人收到停车指令）和 systemd 下的权限限制，**尚未连接实机**。
