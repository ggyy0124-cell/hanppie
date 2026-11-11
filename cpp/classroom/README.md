# 课堂演示服务

供老师上课演示：在和机器人同一网络的 Ubuntu 电脑上运行一个小服务，把 C++ 程序和输入发给它，服务用 `hanppie.h` 编译并运行，实时返回程序输出，可以随时停止。编译选项与 Dev-C++ 一致（`-O2 -std=c++11`，另加 Linux 线程库需要的 `-pthread`），同一份代码两边都能用。

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

服务只提供下面的接口，不带网页。样例网页 [`examples/classroom.html`](../examples/classroom.html) 调用发布它的电脑上的服务（网页地址的主机，8000 端口；直接打开文件时为本机）：选样例或直接写程序，需要时填输入，点「运行」实时看输出，点「停止」随时停下（Ctrl+Enter 也能运行）。把它发布到这台电脑上的静态网站，在这台电脑的浏览器里打开即可；要在教室其他设备上打开，按下面「访问范围」开放局域网访问。

网页样例从已确认的 C++ 挑战卡和预置节目生成；修改源样例或收录范围后运行 `python cpp/tools/generate_classroom_samples.py`，用 `--check` 检查网页是否同步。确认范围与未验证边界见[实机记录](../../docs/records/device/cpp-samples-validation-2026-11-11.md)。

生成单头文件后，用 `python cpp/tools/check_classroom_samples.py --header <hanppie.h 路径>` 编译网页实际收录的全部程序（不运行机器人）；可用 `--compiler` 指定编译器。CI 在 Linux 上执行该检查，覆盖 POSIX 头文件与样例辅助函数的名称冲突。

命令行用法：

```bash
curl -N -F code=@demo.cpp http://127.0.0.1:8000/run                     # 编译并运行，实时显示输出
curl -N -F code=@sum.cpp -F input=@input.txt http://127.0.0.1:8000/run  # 附带输入
curl -N -F code=@sum.cpp -F 'input=3 10 20 30' http://127.0.0.1:8000/run # 输入直接写在命令里
curl -X POST http://127.0.0.1:8000/stop                                 # 停止
curl http://127.0.0.1:8000/status                                       # {"state": "idle"} 或 "running"
```

| 接口 | 说明 |
| --- | --- |
| `POST /run` | 表单字段 `code` 是 C++ 源码，可选字段 `input` 是程序读取的输入（`cin`、`scanf`），不填时程序读不到输入；整个请求不超过 256 KB。编译失败返回 422 和编译器输出；成功则返回 200，实时输出程序内容，最后一行是 `[runner] ...` 说明结束原因。已有程序在运行时返回 409 |
| `POST /stop` | 停止当前程序，返回 `{"stopped": true}`；没有程序在运行时为 `false`。在 `curl` 里按 Ctrl+C 断开 `/run` 也会停止 |
| `GET /status` | 空闲还是正在运行 |

- **停止方式**：先发 SIGTERM，SDK 收到后先停车再退出；2 秒内没退出就强制结束。单次运行最长 3 分钟（`--run-seconds` 可改），输出最多 1 MB，超出都会停止程序。
- **机器人**：程序里用入门接口的 `connect()` 或 `RobotCtrl rm;` 自动发现机器人，不需要配置 IP 和 AppID。官方 App 要先关掉。
- **浏览器请求**：只接受 `--allow-origin` 允许的来源，其他网页的请求返回 403，以免这台电脑上打开的任意网页偷偷运行代码；`curl` 等命令行工具不受影响。随附的 systemd 配置允许 `null`，即隔离（sandbox）页面，供发布平台上的样例网页调用。浏览器分不出隔离页面来自哪个网站，所以这台电脑的浏览器里打开的任意网站的隔离页面也能调用服务，这台电脑的浏览器最好只用于上课。网页放在局域网地址（如 `192.168.x.x`）或本机地址的网站上时，Chrome 不会弹窗询问本地网络访问；放在公网网站上则会。
- **访问范围**：默认只监听本机（`--host 127.0.0.1`）。要让教室里的其他设备访问，加 `--host 0.0.0.0 --allow-client <网段>`，例如 `--allow-client 192.168.5.0/24`（可重复）；本机始终可以访问，其他地址的连接直接断开。开放后，这个网段里的任何设备，以及这些设备的浏览器打开的任意网站的隔离页面，都能在这台电脑上运行代码，只适合可信的教室网络。这台电脑开了防火墙时还要放行 8000 端口。

已用模拟机器人验证编译、运行、停止（停止后机器人收到停车指令）和 systemd 下的权限限制，样例网页已在 Chrome 中以隔离页面验证（包括从局域网地址打开并调用服务），**尚未连接实机**。
