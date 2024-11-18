def TableCoordinate(self, table_num):
    table_coords = {
        1: (1.07, 1.28),
        2: (1.03, 0.17),
        3: (1.07, -0.95),
        4: (-0.12, 1.22),
        5: (-0.06, 0.17),
        6: (-0.10, -0.97),
        7: (-1.16, 1.24),
        8: (-1.17, 0.13),
        9: (-1.14, -0.89)
    }
    return table_coords.get(table_num, None)