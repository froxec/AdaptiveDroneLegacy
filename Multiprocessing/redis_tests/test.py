import redis
import json
import time

r = redis.Redis(host="127.0.0.1", port=6379, db=0)

student = \
        {
            "name": "Alex",
            "surname": "Fox",
            "age": "18",
            "university": "Stanford",
            "grades": {
                "math": 100,
                "physics": 90,
                "informatics": 100
            }
        }


r.set("student", json.dumps(student))
t1 = time.time()
r.get("student")
print(time.time() - t1)