"""Bounded read-only SSH check with explicit failure stages."""
import json,re,socket,subprocess,time

def check_ssh(root):
    config=root/'config/readonly_ssh_config'
    ip=re.search(r'HostName\s+(\S+)',config.read_text(encoding='utf8')).group(1)
    result={'ok':False,'ip':ip,'checked_unix':time.time()}
    try:
        with socket.create_connection((ip,22),timeout=4):pass
    except OSError:
        result.update(stage='tcp',message='无法访问 '+ip+':22；请将电脑和机器狗连接同一热点，并确认机器人当前 IP。尚未进入 SSH 密钥认证。')
        return save(root,result)
    try:
        r=subprocess.run(['ssh','-v','-F',str(config),'-o','ConnectTimeout=8','cyberdog-current-nx','hostname'],capture_output=True,text=True,encoding='utf8',errors='replace',timeout=20)
        error=r.stderr.lower()
        if r.returncode==0 and r.stdout.strip()=='mi-desktop':
            result.update(ok=True,stage='ready',message='SSH 已登录 '+ip+'；机器人身份核验通过')
        elif 'host key verification failed' in error or 'remote host identification has changed' in error:
            result.update(stage='host_key',message='机器人 SSH 身份校验失败；需要核对主机公钥，不能自动跳过')
        elif 'permission denied' in error:
            result.update(stage='authentication',message='SSH 端口可达，但密钥认证失败；请检查机器人授权密钥')
        else:
            result.update(stage='ssh',message='SSH 端口可达，但登录或机器人身份核验失败；请查看连接诊断记录')
        result['detail']=r.stderr[-5000:]
    except subprocess.TimeoutExpired as exc:
        trace=exc.stderr or b''
        if isinstance(trace,bytes):trace=trace.decode('utf8','replace')
        authenticated='Authenticated to ' in trace
        result.update(stage='remote_command' if authenticated else 'handshake',message='SSH 已认证，但机器人命令响应超时' if authenticated else 'SSH 端口可达，但握手或认证超过20秒；请检查热点稳定性与机器人状态',detail=trace[-5000:])
    return save(root,result)

def save(root,result):
    folder=root/'evidence';folder.mkdir(exist_ok=True)
    with (folder/'ssh_checks.jsonl').open('a',encoding='utf8') as f:f.write(json.dumps(result,ensure_ascii=False)+'\n')
    return result
