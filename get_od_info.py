import redis
import json
import time
import cv2
import numpy as np
import base64

r = redis.StrictRedis(host="127.0.0.1", port=6379)
sub = r.pubsub()
sub.subscribe('od_info')

while True:

    for message in sub.listen():
        if message is not None and isinstance(message, dict):
            data = message.get('data')
            try:
                data = data.decode("utf-8")
                data = json.loads(data)
                print(data)
            except:
                print("exception")
                continue

            image = data.get("img")
            image = base64.b64decode(image)
            image = np.frombuffer(image, np.uint8)
            nparr = cv2.imdecode(image, 1)
            cv2.imshow("hello", nparr)
            cv2.waitKey(1)

