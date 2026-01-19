import math


def calculate_metrics(history):
    if not history or len(history) < 2:
        return {"avgErrorPercent": 0.0, "stdDev": 0.0, "minDistance": 0.0, "maxDistance": 0.0}

    recent_data = history[-35:]

    errors = [abs(d["error"]) for d in recent_data]
    setpoints = [d["setpoint"] for d in recent_data]
    distances = [d["filtered"] for d in recent_data]

    if recent_data and "avg_error" in recent_data[-1]:
        avg_error = recent_data[-1]["avg_error"]
    else:
        avg_error = sum(errors) / len(errors) if errors else 0.0

    avg_error_percent = (avg_error / 250.0) * 100.0

    mean_error_signed = sum([d["error"] for d in recent_data]) / len(recent_data)
    variance_sum = sum([pow(d["error"] - mean_error_signed, 2) for d in recent_data])
    variance = variance_sum / len(recent_data)
    std_dev = math.sqrt(variance)

    return {"avgErrorPercent": round(avg_error_percent, 1), "avgError": round(avg_error, 1), "stdDev": round(std_dev, 1), "minDistance": min(distances), "maxDistance": max(distances)}
