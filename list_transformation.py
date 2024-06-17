"""
#######################################
######### List Transformation #########
#######################################

This Python script defines a function `list_transformation` that transforms a list of angles in degrees
by converting each angle to radians, adding π to each radian value, and then dividing the result by 2π.
The script applies this transformation to multiple example lists representing different control and position states,
and prints the transformed values as strings with each value formatted to ten decimal places.
"""

# Import
import math

def list_transformation(list):
    # Values to radians
    radians = [math.radians(x) for x in list]
    # Add pi at each value and divide by 2pi
    result = [(x + math.pi) / (2 * math.pi) for x in radians]
    # Convert to .xml format
    result_str = " ".join(f"{x:.10f}" for x in result)
    return result_str

# List of the Ure3 position
# Important: In case of having more than 300, we need to rest 360
data = {
    "initial": [137.31, -81.60, 69.47, -78.15, -90.49, 317.60-360],
    "place_1": [134.55, -44.95, 46.43, -94.69, -87.18, 313.84-360],
    "place_reg_control_1": [134.94, -49.26, 46.41, -92.56, -87.37, 313.82-360],
    "pick_2": [135.69, -65.77, 80.57, -103.46, -87.71, 315.08-360],
    "pick_reg_control_2": [135.72, -73.32, 80.58, -100.31, -87.09, 314.98-360],
    "pick_1": [135.90, -44.96, 46.32, -94.5, -89.97, 314.39-360],
    "pick_reg_control_1": [135.90, -46.30, 46.29, -93.42, -90.14, 314.39-360],
    "place_2": [128.97, -60.06, 71.69, -103.63, -88.91, 307.77-360],
    "place_reg_2": [128.98, -61.58, 71.69, -102.89, -88.91, 306.93-360]
    }
# Transform and print the results
for name, values in data.items():
    transformed_values = list_transformation(values)
    print(f"{name} = {transformed_values}")

