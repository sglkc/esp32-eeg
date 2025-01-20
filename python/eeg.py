import socketio
import serial
import copy
from datetime import datetime
import csv
from time import sleep

# WebSocket client
sio = socketio.Client(ssl_verify=False)
sio.connect('wss://localhost:8080')

# Serial port configuration
ser = serial.Serial('/dev/ttyACM0', 115200)

SYNC = 0xAA
EXCODE = 0x55
POWER = 0x02
ATTENTION = 0x04
MEDITATION = 0x05
BLINK = 0x16
RAW_WAVE = 0x80
EEG_POWER = 0x83

packet = []
payload = []
generatedChecksum = 0
checksum = 0

data = {
    'timestamp': 0,
    'signal': 0,
    'attention': 0,
    'meditation': 0,
    # 'blink': 0,
    'delta': 0,
    'theta': 0,
    'low_alpha': 0,
    'high_alpha': 0,
    'low_beta': 0,
    'high_beta': 0,
    'low_gamma': 0,
    'mid_gamma': 0,
}

lastData = copy.deepcopy(data)

def create_csv_writer():
    # Create a CSV file with a timestamped name
    timestamp = datetime.now().strftime("%d_%H%M%S")
    csv_filename = f"data_{timestamp}.csv"

    # Open the CSV file for writing
    csv_file = open('datasets/' + csv_filename, mode='w', newline='')
    writer = csv.DictWriter(csv_file, fieldnames=data.keys())
    writer.writeheader()

    return csv_file, writer

csv_file, writer = create_csv_writer()

def resetData():
    global packet, payload, checksum, generatedChecksum
    packet = []
    payload = []
    checksum = 0
    generatedChecksum = 0

def parsePayload(payload):
    global data
    parsed = 0
    code = 0
    length = 0

    while parsed < len(payload):
        extendedCodeLevel = 0

        while payload[parsed] == EXCODE:
            extendedCodeLevel += 1
            parsed += 1

        code = payload[parsed]
        parsed += 1

        if code & 0x80:
            length = payload[parsed]
            parsed += 1
        else:
            length = 1

        if code == EEG_POWER:
            data['delta'] = (payload[parsed] & 0xFF << 16) | (payload[parsed + 1] & 0xFF << 8) | payload[parsed + 2]
            data['theta'] = (payload[parsed + 3] & 0xFF << 16) | (payload[parsed + 4] & 0xFF << 8) | payload[parsed + 5]
            data['low_alpha'] = (payload[parsed + 6] & 0xFF << 16) | (payload[parsed + 7] & 0xFF << 8) | payload[parsed + 8]
            data['high_alpha'] = (payload[parsed + 9] & 0xFF << 16) | (payload[parsed + 10] & 0xFF << 8) | payload[parsed + 11]
            data['low_beta'] = (payload[parsed + 12] & 0xFF << 16) | (payload[parsed + 13] & 0xFF << 8) | payload[parsed + 14]
            data['high_beta'] = (payload[parsed + 15] & 0xFF << 16) | (payload[parsed + 16] & 0xFF << 8) | payload[parsed + 17]
            data['low_gamma'] = (payload[parsed + 18] & 0xFF << 16) | (payload[parsed + 19] & 0xFF << 8) | payload[parsed + 20]
            data['mid_gamma'] = (payload[parsed + 21] & 0xFF << 16) | (payload[parsed + 22] & 0xFF << 8) | payload[parsed + 23]
        # elif code == BLINK:
        #     data['blink'] = payload[parsed]
        elif code == ATTENTION:
            data['attention'] = payload[parsed]
        elif code == MEDITATION:
            data['meditation'] = payload[parsed]
        elif code == POWER:
            data['signal'] = 200 - payload[parsed]

        parsed += length

    return data

def handleData(byte):
    global packet, payload, generatedChecksum, checksum, lastData

    packet.append(byte)

    if len(packet) < 3: return

    sync0, sync1, payloadLength = packet[:3]

    if sync0 != SYNC:
        packet = []
        return
    if sync0 != SYNC and sync1 != SYNC:
        packet = []
        return
    if sync0 == SYNC and sync1 == SYNC and payloadLength == SYNC:
        packet = []
        return

    if len(packet) > 3 and len(payload) < payloadLength:
        generatedChecksum += byte
        payload.append(byte)

    if len(packet) == payloadLength + 4:
        generatedChecksum = ~(generatedChecksum & 0xFF) & 0xFF
        checksum = byte

        if generatedChecksum != checksum:
            print('Checksum error', flush=True)
            resetData()
            return

        parsePayload(payload)

        duplicate = all(data[key] == lastData[key] for key in data)

        if not duplicate:
            send_data()

        lastData = copy.deepcopy(data)

        resetData()

def send_data():
    data['timestamp'] = int(datetime.now().timestamp())
    print(data['timestamp'], flush=True)
    print(data, flush=True)
    writer.writerow(data)
    del data['timestamp']

    sio.emit('eeg', data)

    mean = max(data['attention'], data['meditation'])

    if data['signal'] < 100: return

    # if mean == 0:
    #     command = 'stop'
    if mean <= 33:
        command = 'stop'
    elif mean <= 66:
        command = 'backward'
    elif mean <= 100:
        command = 'forward'
    else:
        command = 'stop'

    # if len(command) > 0:
    #     print(command)
    #     sio.emit('esp32/commands', command)


try:
    while True:
        # sio.emit('eeg', data)
        # sleep(3)
        if ser.in_waiting == 0: continue

        byte = ser.read(1)[0]
        handleData(byte)
except KeyboardInterrupt:
    print('Stopping...', flush=True)
finally:
    csv_file.close()
