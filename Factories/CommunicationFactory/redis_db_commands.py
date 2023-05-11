import redis
from typing import Type
def redis_stream_add(redis_db: Type[redis.Redis],
                        stream: str,
                        data: list):
    data_dict = {}
    for id, value in enumerate(data):
        data_dict[str(id)] = float(value)
    redis_db.xadd(stream, data_dict)

def unpack_list(func):
    def wrap(*args, **kwargs):
        received_streams = func(*args, **kwargs)
        unpacked_stream = received_streams[0]
        data_dict = unpacked_stream[1][0][1]
        data_list = []
        for key, value in data_dict.items():
            data_list.append(float(value))
        return data_list
    return wrap

@unpack_list
def redis_stream_read_last(redis_db: Type[redis.Redis],
                            stream: str):
    received_streams = redis_db.xread({stream: '$'}, None, 0)
    return received_streams