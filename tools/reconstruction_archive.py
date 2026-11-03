"""Size-bounded sequential raw frame archive, for exact accepted-frame replay."""
import json


class FrameArchive:
    def __init__(self, path, max_bytes=64*1024*1024):
        self.path = path
        self.max_bytes = max_bytes
        self.frames = self.bytes = 0
        self.truncated = False
        with self.path.open('xb'):
            pass

    def append(self, record):
        if self.truncated:
            return
        data = (json.dumps(record, separators=(',', ':'), allow_nan=False)+'\n').encode('utf-8')
        if self.max_bytes is not None and self.bytes+len(data) > self.max_bytes:
            self.truncated = True
            return
        with self.path.open('ab') as stream:
            stream.write(data)
        self.bytes += len(data)
        self.frames += 1

    def stats(self, accepted_frames):
        return {'file':self.path.name, 'frames':self.frames, 'bytes':self.bytes,
                'max_bytes':self.max_bytes, 'truncated':self.truncated,
                'complete':not self.truncated and self.frames == accepted_frames}
