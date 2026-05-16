import sigrokdecode as srd


class Decoder(srd.Decoder):
    api_version = 3
    id = 'unio'
    name = 'UNI/O'
    longname = 'Microchip UNI/O'
    desc = 'Single-wire Manchester-encoded serial bus.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = []
    tags = ['Embedded/industrial']
    channels = (
        {'id': 'scio', 'name': 'SCIO', 'desc': 'Serial data / clock line'},
    )
    options = ()
    annotations = (
        ('standby', 'Standby pulse'),
        ('start', 'Start header'),
        ('preamble', 'Preamble'),
        ('byte', 'Byte'),
        ('data', 'Data byte'),
        ('mak', 'Master acknowledge'),
        ('sak', 'Slave acknowledge'),
        ('bit', 'Bit'),
        ('error', 'Error'),
    )
    annotation_rows = (
        ('frames', 'Frame', (0, 1, 2)),
        ('bytes', 'Bytes', (3, 4)),
        ('acks', 'Acknowledge', (5, 6)),
        ('bits', 'Bits', (7,)),
        ('errors', 'Errors', (8,)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.out_ann = None
        self.bit_width = 0.0
        self.edge_tolerance = 0.0
        self.min_pulse = 1

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value
            # Reject narrow spikes that are far below the minimum UNI/O pulse width.
            self.min_pulse = max(1, int(value / 2000000.0))

    def putx(self, ss, es, ann, texts):
        self.put(int(ss), int(es), self.out_ann, [ann, texts])

    def wait_edge(self):
        pins = self.wait({0: 'e'})
        return int(self.samplenum), int(pins[0])

    def wait_stable_edge(self):
        while True:
            pins = self.wait({0: 'e'})
            edge_sample = int(self.samplenum)
            edge_level = int(pins[0])
            pins = self.wait([{0: 'e'}, {'skip': self.min_pulse}])
            if self.matched[1]:
                return edge_sample, edge_level

    def find_start_header(self):
        older_edge = None
        last_edge = None

        while True:
            edge_sample, edge_level = self.wait_stable_edge()

            if last_edge is None:
                last_edge = (edge_sample, edge_level)
                continue

            older_edge = last_edge if older_edge is None else older_edge
            prev_edge = last_edge
            last_edge = (edge_sample, edge_level)

            if prev_edge[1] != 0 or edge_level != 1:
                older_edge = prev_edge
                continue

            low_width = edge_sample - prev_edge[0]
            if low_width < self.min_pulse:
                older_edge = prev_edge
                continue

            if older_edge is None or older_edge[1] != 1:
                older_edge = prev_edge
                continue

            high_width = prev_edge[0] - older_edge[0]
            if high_width < (low_width * 8):
                older_edge = prev_edge
                continue

            return prev_edge[0], edge_sample

    def wait_until(self, target):
        delta = int(round(target - self.samplenum))
        if delta < 0:
            delta = 0
        pins = self.wait({'skip': delta})
        return int(self.samplenum), int(pins[0])

    def calc_timing(self, start_sample, start_edge, mid_edges):
        if len(mid_edges) < 8:
            return False

        first_half = mid_edges[0][0] - start_edge
        if first_half <= 0:
            return False

        full_periods = []
        for index in range(1, len(mid_edges)):
            full_periods.append(mid_edges[index][0] - mid_edges[index - 1][0])

        if not full_periods:
            return False

        bit_width = float(sum(full_periods)) / float(len(full_periods))
        if bit_width <= 0:
            return False

        tolerance = bit_width * 0.30
        if abs((first_half * 2.0) - bit_width) > tolerance:
            return False

        for period in full_periods:
            if abs(period - bit_width) > tolerance:
                return False

        expected_levels = [0, 1, 0, 1, 0, 1, 0, 1]
        for index, edge in enumerate(mid_edges[:8]):
            if edge[1] != expected_levels[index]:
                return False

        self.bit_width = bit_width
        self.edge_tolerance = max(1.0, bit_width * 0.35)
        return True

    def wait_for_mid_edge(self, bit_start):
        target = bit_start + (self.bit_width / 2.0)
        window_start = target - self.edge_tolerance
        window_end = target + self.edge_tolerance

        self.wait_until(window_start)
        while int(self.samplenum) <= int(window_end):
            remaining = int(round(window_end - self.samplenum))
            if remaining < 0:
                break
            pins = self.wait([{0: 'e'}, {'skip': remaining}])
            level = int(pins[0])
            if not self.matched[0]:
                break

            edge_sample = int(self.samplenum)
            pins = self.wait([{0: 'e'}, {'skip': self.min_pulse}])
            if self.matched[1]:
                return edge_sample, level, True

        return int(self.samplenum), 0, False

    def decode_bit(self, bit_start):
        edge_sample, level, found = self.wait_for_mid_edge(bit_start)
        bit_end = bit_start + self.bit_width
        if not found:
            self.putx(bit_start, bit_end, 8, ['Missing mid-bit edge', 'Bit error', 'Err'])
            return None, bit_end

        bit_value = 1 if level == 1 else 0
        bit_end = edge_sample + (self.bit_width / 2.0)
        self.putx(bit_start, bit_end, 7, [str(bit_value)])
        return bit_value, bit_end

    def decode_byte(self, byte_start, ann_idx, prefix):
        value = 0
        current = byte_start

        for _ in range(8):
            bit_value, current = self.decode_bit(current)
            if bit_value is None:
                return None, current
            value = (value << 1) | bit_value

        text = '{} 0x{:02X}'.format(prefix, value)
        self.putx(byte_start, current, ann_idx, [text, '0x{:02X}'.format(value)])
        return value, current

    def decode_ack(self, bit_start, ann_idx, active_text, inactive_text, short_active, short_inactive):
        bit_value, bit_end = self.decode_bit(bit_start)
        if bit_value is None:
            return None, bit_end
        if bit_value:
            self.putx(bit_start, bit_end, ann_idx, [active_text, short_active])
        else:
            self.putx(bit_start, bit_end, ann_idx, [inactive_text, short_inactive])
        return bit_value, bit_end

    def decode_sak(self, bit_start):
        bit_end = bit_start + self.bit_width
        edge_sample, level, found = self.wait_for_mid_edge(bit_start)
        if found and level == 1:
            bit_end = edge_sample + (self.bit_width / 2.0)
            self.putx(bit_start, bit_end, 6, ['SAK / ACK', 'SAK'])
            self.putx(bit_start, bit_end, 7, ['1'])
            return 1, bit_end

        self.putx(bit_start, bit_end, 6, ['NoSAK / NACK', 'NoSAK'])
        return 0, bit_end

    def decode(self):
        if not self.samplerate:
            raise Exception('Cannot decode without samplerate.')

        while True:
            standby_start, start_edge = self.find_start_header()

            mid_edges = []
            for _ in range(8):
                sample, level = self.wait_stable_edge()
                mid_edges.append((sample, level))

            if not self.calc_timing(standby_start, start_edge, mid_edges):
                self.putx(standby_start, mid_edges[-1][0], 8,
                          ['Invalid preamble timing', 'Timing error', 'Err'])
                continue

            self.putx(standby_start, start_edge, 0,
                      ['Standby pulse', 'Standby', 'SBY'])
            self.putx(standby_start, start_edge, 1,
                      ['Start header', 'Start', 'S'])
            self.putx(start_edge, int(start_edge + (8.0 * self.bit_width)), 2,
                      ['Preamble 0x55', 'Preamble', 'Pre'])

            current = float(start_edge) + (8.0 * self.bit_width)

            mak, current = self.decode_ack(
                current, 5,
                'MAK / continue', 'NoMAK / stop', 'MAK', 'NoMAK')
            if mak is None:
                continue
            self.decode_sak(current)
            current += self.bit_width

            while True:
                byte_value, current = self.decode_byte(current, 3, 'Byte')
                if byte_value is None:
                    break

                mak, current = self.decode_ack(
                    current, 5,
                    'MAK / continue', 'NoMAK / stop', 'MAK', 'NoMAK')
                if mak is None:
                    break
                self.decode_sak(current)
                current += self.bit_width
                if not mak:
                    break
