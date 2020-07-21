

from ModularEvoManager import GazeboMessageSubscriber
import asyncio
import sys

print("Execution with the port:")
port = int(sys.argv[1]) if int(sys.argv[1]) else 11345
print(port)

##Create a new file in which to write


Subscriber = GazeboMessageSubscriber("localhost", port)
loop = asyncio.get_event_loop()
loop.run_until_complete(Subscriber.connect())

## 45 = DIRECT ENCODING, 46 = ACK
## 125 = INDIRECT ENCODING, 126 = ACK
## 86 = DIRECT ENCODING MODULAR, 87 = ACK
## 101 = INDIRECT ENCODING MODULAR, 102 = ACK

experiment = int(sys.argv[2])
continueLearning = 0
loop.run_until_complete(Subscriber.executeTest(experiment,continueLearning))

