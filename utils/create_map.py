from dataclasses import dataclass

width = 181
height = 231
# width = 4
# height = 2
max_val = 255
filename = "map.pgm"

@dataclass
class Coordinate:
    x: int
    y: int

@dataclass
class Line:
    color: bytes
    start: Coordinate
    end: Coordinate

def main():
    ok = b'\xEF'
    ng = b'\x00'
    or_not = b'\xCD'

    # Initialize flat pixel buffer (row-major)
    data = [ok for _ in range(width * height)]

    # Define lines to draw
    line_list = [
        Line(ng, Coordinate(0, 0), Coordinate(50, 0)),
        Line(ng, Coordinate(0, 0), Coordinate(0, 50)),
        Line(ng, Coordinate(0, 51), Coordinate(0, 61)),
        Line(ng, Coordinate(0, 91), Coordinate(0, 126)),
        Line(ng, Coordinate(0, 156), Coordinate(0, 191)),
        Line(ng, Coordinate(0, 221), Coordinate(0, 230)),
        Line(ng, Coordinate(50, 0), Coordinate(50, 50)),
        Line(ng, Coordinate(50, 50), Coordinate(180, 50)),
        Line(ng, Coordinate(180, 50), Coordinate(180, 230)),
        Line(ng, Coordinate(0, 230), Coordinate(180, 230)),
        *[Line(or_not, Coordinate(x_, 90), Coordinate(x_, 90 + 90)) for x_ in range(60, 60 + 28 + 1)],
        Line(ng, Coordinate(60, 90), Coordinate(60, 90 + 90)),
        Line(ng, Coordinate(60 + 28 + 1, 90), Coordinate(60 + 28 + 1, 90 + 90)),
        Line(ng, Coordinate(60, 90), Coordinate(60 + 28 + 1, 90)),
        Line(ng, Coordinate(60, 90+90), Coordinate(60 + 28 + 1, 90+90)),
        *[Line(or_not, Coordinate(51, y_), Coordinate(180, y_)) for y_ in range(0, 50)]
    ]

    # Draw each line
    for line_ in line_list:
        draw_line(data, line_)

    # Write PGM with vertical flip
    with open(filename, "wb") as f:
        header = f"P5\n{width} {height}\n{max_val}\n"
        f.write(header.encode("ascii"))

        # Write rows in reverse order to flip vertically
        for y in range(height - 1, -1, -1):
            start = y * width
            end = start + width
            # join bytes for this row
            f.write(b''.join(data[start:end]))


def draw_line(data: list, line: Line):
    # Only horizontal or vertical lines
    assert line.start.x == line.end.x or line.start.y == line.end.y, "Only horizontal or vertical lines supported"

    if line.start.x == line.end.x:
        x_ = line.start.x
        for y_ in range(line.start.y, line.end.y + 1):
            set_grid(data, line.color, x_, y_)
    else:
        y_ = line.start.y
        for x_ in range(line.start.x, line.end.x + 1):
            set_grid(data, line.color, x_, y_)


def set_grid(data: list, color: bytes, x: int, y: int):
    # Compute flat index and assign color
    idx = y * width + x
    data[idx] = color

if __name__ == "__main__":
    main()