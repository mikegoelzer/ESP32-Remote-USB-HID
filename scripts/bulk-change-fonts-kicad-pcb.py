import re

# Define the path to the Kicad PCB file
kicad_pcb_file = '../USBHIDKey/USBHIDKey.kicad_pcb'

# Define the new font properties
new_font_height_mils = 30
new_font_width_mils = 30
new_font_thickness_mils = 6

print(f"This script resizes all fonts for parts starting with R, D, or C to:\n  {new_font_width_mils} mils width\n  {new_font_height_mils} mils height\n  {new_font_thickness_mils} mils thickness")
print(f"Operates on:  '{kicad_pcb_file}'")
print("Overwrites the original file")

# Regular expressions to match the font size and thickness lines
font_size_re = re.compile(r'\(size [0-9.]+ [0-9.]+\)')
font_thickness_re = re.compile(r'\(thickness [0-9.]+\)')

# Regular expression to match the reference designators
ref_designator_re = re.compile(r'\(property "Reference" "([RDC][0-9]+)"')

# Layers to be updated
layers_to_update = {'F.SilkS', 'B.SilkS', 'F.Fab', 'B.Fab'}

def mils_to_mm(mils):
    return mils * 0.0254

# Function to update the font properties in a line
def update_font_properties(line):
    line = font_size_re.sub(f'(size {mils_to_mm(new_font_width_mils):.4f} {mils_to_mm(new_font_height_mils):.4f})', line)
    line = font_thickness_re.sub(f'(thickness {mils_to_mm(new_font_thickness_mils):.4f})', line)
    return line

# Read the PCB file
with open(kicad_pcb_file, 'r') as file:
    lines = file.readlines()

# Flag to indicate if we are inside a part that needs updating
inside_target_part = False

# Flags to indicate if we are inside an effects block and a font block
inside_effects_block = False
inside_font_block = False

# Update the lines with new font properties
updated_lines = []
ref_designator = ""
for line in lines:
    #print(f"Processing line: {line.strip()}")  # Debug print statement

    match = ref_designator_re.search(line)
    if match:
        inside_target_part = True
        ref_designator = match.group(1)
        print(f"Found reference designator: {ref_designator}")  # Debug print statement
    # Flag to indicate if we are inside a footprint block
    inside_footprint_block = False

    if '(footprint' in line:
        inside_footprint_block = True
        print("Entering footprint block")  # Debug print statement

    if inside_footprint_block and ref_designator_re.search(line):
        inside_target_part = True
        ref_designator = ref_designator_re.search(line).group(1)
        print(f"  Entering matching part block: {ref_designator}")  # Debug print statement

    if inside_target_part:
        # Check if the line contains a layer we care about
        if '(layer' in line:
            if any(f'(layer "{layer}")' in line for layer in layers_to_update):
                inside_modify_layer = True
                print(f"    Inside modifiable layer: {line.strip()}")  # Debug print statement
            else:
                inside_modify_layer = False
                print(f"    Inside non-modifiable layer: {line.strip()}")  # Debug print statement

        if '(effects' in line:
            inside_effects_block = True
            print(f"    Entering effects block for {ref_designator}")  # Debug print statement

        if inside_effects_block and '(font' in line:
            inside_font_block = True
            print(f"      Entering font block for {ref_designator}")  # Debug print statement

        if inside_font_block and ('(size' in line or '(thickness' in line):
            if inside_modify_layer:
                print(f"        Updating font properties for line: {line.strip()}")  # Debug print statement
                line = update_font_properties(line)
            else:
                print(f"        Skipping font update in non-modifiable layer: {line.strip()}")  # Debug print statement

        if inside_font_block and line.strip() == ')':
            inside_font_block = False
            print(f"      End of font block for {ref_designator}")  # Debug print statement

        elif inside_effects_block and line.strip() == ')':
            inside_effects_block = False
            print(f"    End of effects block for {ref_designator}")  # Debug print statement

        elif inside_target_part and line.strip() == ')':
            inside_target_part = False
            print(f"  End of part block for {ref_designator}")  # Debug print statement

    elif inside_footprint_block and line.strip() == ')':
        inside_footprint_block = False
        print(f"Exiting footprint block for {ref_designator}")  # Debug print statement

    updated_lines.append(line)

# Write the updated lines back to the PCB file
with open(kicad_pcb_file, 'w') as file:
    file.writelines(updated_lines)

print("Silkscreen and Fab font properties updated successfully for parts starting with R, D, or C.")