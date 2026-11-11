"""Start and monitor the three loopback web services. Never starts robot motion."""
import socket,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
SERVICES={8765:'map_viewer.py',8766:'reconstruction_server.py',8768:'reconstruction_workbench.py'}
def listening(port):
    with socket.socket() as sock:
        sock.settimeout(.3)
        return sock.connect_ex(('127.0.0.1',port))==0
def main():
    # A loopback lock prevents duplicate supervisors.
    lock=socket.socket()
    try:lock.bind(('127.0.0.1',18765))
    except OSError:return
    processes={}
    while True:
        for port,script in SERVICES.items():
            process=processes.get(port)
            if listening(port) or (process and process.poll() is None):continue
            log=(ROOT/'evidence'/('service_'+str(port)+'.log')).open('a',encoding='utf8')
            try:
                processes[port]=subprocess.Popen([sys.executable,'-B',str(ROOT/'tools'/script)],
                    cwd=str(ROOT),stdout=log,stderr=log)
            finally:log.close()
        time.sleep(5)
if __name__=='__main__':main()
