import io from 'socket.io-client'
import { SerialPort } from 'serialport';
import { ByteLengthParser } from '@serialport/parser-byte-length';

const ws = io('ws://localhost:8080')

const port = new SerialPort({
  path: '/dev/ttyACM0',
  baudRate: 115200
});

const parser = port.pipe(new ByteLengthParser({ length: 1 }))

const getHex = (c) => c.toString(16).padStart(2, 0)

const printHex = (...bytes) => console.log(
  new Uint8Array(bytes).reduce((s, c) => s + getHex(c) + ' ', '')
)

const SYNC = 0xAA
const EXCODE = 0x55
const POWER = 0x02
const ATTENTION = 0x04
const MEDITATION = 0x05
const BLINK = 0x16
const RAW_WAVE = 0x80
const EEG_POWER = 0x83

let packet = []
let payload = []
let generatedChecksum = 0
let checksum = 0

const data = {
  signal: 0,
  attention: 0,
  meditation: 0,
  // blink: 0,
  delta: 0,
  theta: 0,
  low_alpha: 0,
  high_alpha: 0,
  low_beta: 0,
  high_beta: 0,
  low_gamma: 0,
  mid_gamma: 0,
}

let lastData = data

function resetData() {
  packet = []
  payload = []
  checksum = 0
  generatedChecksum = 0
}

function parsePayload(payload) {
  let parsed = 0, code = 0, length = 0

  while (parsed < payload.length) {
    let extendedCodeLevel = 0

    while (payload[parsed] == EXCODE) {
      extendedCodeLevel++
      parsed++
    }

    code = payload[parsed++]

    if (code & 0x80)
      length = payload[parsed++]
      else
      length = 1

    // if (code != 0x80) {
    //   console.log(`EXCODE level: ${extendedCodeLevel} CODE: ${code} LENGTH: ${length}`)
    //
    //   for (let i = 0; i < length; i++) {
    //     console.log(payload[parsed+i] & 0xFF)
    //   }
    // }

    switch (code) {
      case EEG_POWER:
        data.delta = (payload[parsed+0] & 0xFF << 16) | (payload[parsed+1] & 0xFF << 8) | payload[parsed+2]
        data.theta = (payload[parsed+3] & 0xFF << 16) | (payload[parsed+4] & 0xFF << 8) | payload[parsed+5]
        data.low_alpha = (payload[parsed+6] & 0xFF << 16) | (payload[parsed+7] & 0xFF << 8) | payload[parsed+8]
        data.high_alpha = (payload[parsed+9] & 0xFF << 16) | (payload[parsed+10] & 0xFF << 8) | payload[parsed+11]
        data.low_beta = (payload[parsed+12] & 0xFF << 16) | (payload[parsed+13] & 0xFF << 8) | payload[parsed+14]
        data.high_beta = (payload[parsed+15] & 0xFF << 16) | (payload[parsed+16] & 0xFF << 8) | payload[parsed+17]
        data.low_gamma = (payload[parsed+18] & 0xFF << 16) | (payload[parsed+19] & 0xFF << 8) | payload[parsed+20]
        data.mid_gamma = (payload[parsed+21] & 0xFF << 16) | (payload[parsed+22] & 0xFF << 8) | payload[parsed+23]
        break
      // case BLINK:
      //   data.blink = payload[parsed]
      //   break
      case ATTENTION:
        data.attention = payload[parsed]
        break
      case MEDITATION:
        data.meditation = payload[parsed]
        break
      case POWER:
        data.signal = /* inverse */ 200 - payload[parsed]
        break
    }

    parsed += length
  }

  return data
}

function handleData([ byte ]) {
  packet.push(byte)
  const [ sync0, sync1, payloadLength ] = packet

  // cek cek
  if (sync0 !== SYNC) {
    return packet = []
  }
  if (sync0 !== SYNC && sync1 !== SYNC) {
    return packet = []
  }
  if (sync0 === SYNC && sync1 === SYNC && payloadLength === SYNC) {
    return packet = []
  }

  // masukkan payload
  if (packet.length > 3 && payload.length < payloadLength) {
    generatedChecksum += byte
    payload.push(byte)
  }

  if (packet.length === payloadLength + 4) {
    generatedChecksum = ~(generatedChecksum & 0xFF) & 0xFF
    checksum = byte

    if (generatedChecksum !== checksum) {
      console.error('checksum error')
      resetData()
      return
    }

    parsePayload(payload)

    const duplicate = Object.keys(data).every((key) => data[key] === lastData[key])

    if (!duplicate) {
      console.log(data)
      ws.emit('eeg', data)
    }

    lastData = structuredClone(data)

    // fetch('http://localhost:8080', {
    //   method: 'post',
    //   headers: {
    //     'content-type': 'application/json'
    //   },
    //   body: JSON.stringify(data)
    // })

    // selesai semua, reset
    resetData()
  }
};

parser.on('data', handleData)

port.on('error', (err) => {
  console.error('Error: ', err.message);
})
