# core_lib/threaded_stream.py
import threading
import queue

class ThreadedStream:
    def __init__(self, reader, queue_size=1):
        self.reader = reader
        self.stopped = False
        self.queue = queue.Queue(maxsize=queue_size)
        self.thread = threading.Thread(target=self._update, daemon=True)

    def start(self):
        self.thread.start()
        print("视频流读取线程已启动。")
        return self

    def _update(self):
        try:
            for frame in self.reader:
                if self.stopped:
                    break
                if not self.queue.empty():
                    try:
                        self.queue.get_nowait()
                    except queue.Empty:
                        pass
                self.queue.put(frame)
        finally:
            if hasattr(self.reader, 'release'):
                self.reader.release()

    def read(self):
        return self.queue.get()

    def more(self):
        return self.thread.is_alive() or not self.queue.empty()

    def stop(self):
        self.stopped = True
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        print("视频流读取线程已停止。")