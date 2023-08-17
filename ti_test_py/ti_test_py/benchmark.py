import time
import threading
import json
import inspect

class ProfileResult:
    def __init__(self, name, start, end, thread_id):
        self.Name = name
        self.Start = start
        self.End = end
        self.ThreadID = thread_id

class InstrumentationTimer:
    def __init__(self):
        self.m_Name = inspect.stack()[1][0].f_code.co_name
        self.start = time.time()
    
    def stop(self):
        self.end = time.time()
        write_in = ProfileResult(name=self.m_Name, start=self.start, end=self.end, thread_id=threading.get_ident())
        Instrumentor.get().WriteProfile(write_in)
    
class Instrumentor:

    json_temp = []

    def BeginSession(self, filepath="result.json" ):
        self.file = open(filepath, "w")
        self.file.truncate(0)
        self.WriteHeader()

    def EndSession(self):
        self.WriteFooter()
        self.file.close()

    def WriteProfile(self, result: ProfileResult):
        Instrumentor.json_temp = [
            {
            "cat":"function", 
            "dur": result.End-result.Start, 
            "name":result.Name, 
            "ph":"X", "pid":0, 
            "tid": result.ThreadID, 
            "ts":result.Start
            },  *Instrumentor.json_temp
        ]
    
    def WriteHeader(self):
        m_OutputStream = "{\"otherData\": {},\"traceEvents\":"
        self.file.write(m_OutputStream)
        self.file.flush()

    def WriteFooter(self):
        m_OutputStream = json.dumps(Instrumentor.json_temp) + "}"
        self.file.write(m_OutputStream)
        self.file.flush()

    @staticmethod
    def get():
        instant = Instrumentor()
        return instant