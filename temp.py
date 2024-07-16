import re

# Define the new font properties
new_font_size = 30
new_font_thickness = 6

# Regular expressions to match the font size and thickness lines
font_size_re = re.compile(r'\(size [0-9.]+ [0-9.]+\)')
font_thickness_re = re.compile(r'\(thickness [0-9.]+\)')

# Regular expression to match the reference designators
ref_designator_re = re.compile(r'\(fp_text reference ([RDC][0-9]+)')

# Layers to be updated
layers_to_update = {'F.SilkS', 'B.SilkS', 'F.Fab', 'B.Fab'}

# Function to update the font properties in a line
def update_font_properties(line):
    line = font_size_re.sub(f'(size {new_font_size} {new_font_size})', line)
    line = font_thickness_re.sub(f'(thickness {new_font_thickness})', line)
    return line

# Read the PCB file
with open('your_pcb_file.kicad_pcb', 'r') as file:
    lines = file.readlines()

# Flag to indicate if we are inside a part that needs updating
inside_target_part = False

# Update the lines with new font properties
updated_lines = []
for line in lines:
    if ref_designator_re.search(line):
        inside_target_part = True
    if inside_target_part and any(layer in line for layer in layers_to_update):
        line = update_font_properties(line)
    if inside_target_part and line.strip() == ')':
        inside_target_part = False
    updated_lines.append(line)

# Write the updated lines back to the PCB file
with open('your_pcb_file.kicad_pcb', 'w') as file:
    file.writelines(updated_lines)

print("Silkscreen and Fab font properties updated successfully for parts starting with R, D, or C.")