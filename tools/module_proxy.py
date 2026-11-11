"""Loopback-only routing for the modeling modules under the main console."""
import http.client
import re

def proxy_module(handler, prefix, port):
    path=handler.path[len(prefix):] or '/'
    length=int(handler.headers.get('Content-Length','0'))
    if length<0 or length>1024*1024:
        handler.send_error(413);return
    body=handler.rfile.read(length) if length else None
    headers={'Origin':'http://127.0.0.1:'+str(port),'X-Console-Proxy':'1'}
    if handler.headers.get('Content-Type'):headers['Content-Type']=handler.headers['Content-Type']
    client=http.client.HTTPConnection('127.0.0.1',port,timeout=100)
    try:
        client.request(handler.command,path,body,headers)
        reply=client.getresponse();data=reply.read()
        content=reply.getheader('Content-Type','application/octet-stream')
        if 'text/html' in content or ('javascript' in content and not path.startswith('/assets/')):
            text=data.decode('utf8')
            text=text.replace('http://127.0.0.1:8765/','/navigation')
            text=text.replace('http://127.0.0.1:8768/','/offline/')
            text=re.sub(r"""(['"`])/(?!/)""",lambda m:m.group(1)+prefix+'/',text)
            # Links between modules must remain on the central router.
            text=text.replace(prefix+'/navigation','/navigation').replace(prefix+'/offline/','/offline/')
            text=text.replace('href="'+prefix+'/">← 返回功能选择','href="/">← 返回功能选择')
            data=text.encode('utf8')
        handler.send_response(reply.status)
        handler.send_header('Content-Type',content)
        handler.send_header('Content-Length',str(len(data)))
        handler.send_header('Cache-Control','no-store')
        handler.end_headers();handler.wfile.write(data)
    except (OSError,http.client.HTTPException):
        data='模块服务尚未就绪，请运行 Start-WebConsole.ps1 启动完整工作台。'.encode('utf8')
        handler.send_response(503);handler.send_header('Content-Type','text/plain; charset=utf-8')
        handler.send_header('Content-Length',str(len(data)));handler.end_headers();handler.wfile.write(data)
    finally:client.close()
