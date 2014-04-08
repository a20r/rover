
import riskgrid

size = 100
mid_x = 320
mid_y = 240
square = riskgrid.RiskGrid()
square.add_risk_point(mid_x - size, mid_y - size, 300, 1)
square.add_risk_point(mid_x + size, mid_y - size, 300, 1)
square.add_risk_point(mid_x - size, mid_y + size, 300, 1)
square.add_risk_point(mid_x + size, mid_y + size, 300, 1)

risk_dict = {
    "square": square
}

def get(val, default):
    return risk_dict.get(val, default)

