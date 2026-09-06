```python
import serial
import csv
import time
from datetime import datetime
from pathlib import Path

SERIAL_PORT = "COM20"
BAUD_RATE = 115200
RECORD_SECONDS = 5 * 60
OUTPUT_FOLDER = Path("exg_recordings")

OUTPUT_FOLDER.mkdir(parents=True, exist_ok=True)

try:
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        timeout=1
    )
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
except serial.SerialException as e:
    print(f"ERROR: Could not open {SERIAL_PORT}")
    print(e)
    raise SystemExit(1)

start_datetime = datetime.now()

filename = f"EXG_{start_datetime.strftime('%Y%m%d_%H%M%S')}.csv"
output_file = OUTPUT_FOLDER / filename

print(f"Saving data to:")
print(output_file.resolve())

start_monotonic = time.perf_counter()

rows_written = 0
header_written = False

csv_file = open(
    output_file,
    "w",
    newline="",
    encoding="utf-8"
)

writer = None

try:
    while True:
        elapsed = time.perf_counter() - start_monotonic

        if elapsed >= RECORD_SECONDS:
            break

        raw_line = ser.readline()

        if not raw_line:
            continue

        try:
            line = raw_line.decode("utf-8").strip()
        except UnicodeDecodeError:
            line = raw_line.decode(
                "utf-8",
                errors="replace"
            ).strip()

        if not line:
            continue

        values = line.split(",")

        if not header_written:
            is_header = False

            for value in values:
                try:
                    float(value)
                except ValueError:
                    is_header = True
                    break

            if is_header:
                incoming_header = values
            else:
                incoming_header = [
                    f"bioamp_{i + 1}"
                    for i in range(len(values))
                ]

            output_header = [
                "timestamp",
                "epoch",
                "elapsed_time_s"
            ] + incoming_header

            writer = csv.writer(csv_file)
            writer.writerow(output_header)
            header_written = True

            if is_header:
                csv_file.flush()
                continue

        now = datetime.now().astimezone()

        timestamp = now.isoformat(
            timespec="milliseconds"
        )

        epoch = time.time()

        elapsed = time.perf_counter() - start_monotonic

        writer.writerow([
            timestamp,
            f"{epoch:.6f}",
            f"{elapsed:.6f}"
        ] + values)

        rows_written += 1

        if rows_written % 100 == 0:
            csv_file.flush()

finally:
    csv_file.flush()
    csv_file.close()
    ser.close()

actual_duration = time.perf_counter() - start_monotonic

print()
print("EXG RECORDING COMPLETE")
print(f"Duration       : {actual_duration:.2f} seconds")
print(f"Samples saved  : {rows_written}")
print(f"Output file    : {output_file.resolve()}")
```
