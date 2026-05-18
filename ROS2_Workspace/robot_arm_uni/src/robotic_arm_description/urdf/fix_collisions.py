import xml.etree.ElementTree as ET

# 1. Load the URDF
input_file = "robotic_arm_stable.urdf.xacro"
output_file = "robotic_arm_stable.urdf.xacro" 

tree = ET.parse(input_file)
root = tree.getroot()

# 2. Define the parts that are overlapping and exploding
problem_links = ['left_gear', 'rigth_gear', 'left_link', 'rigth_link', 'left_finger', 'rigth_finger']

# 3. Find and delete their <collision> tags
for link in root.findall('link'):
    if link.get('name') in problem_links:
        collision = link.find('collision')
        if collision is not None:
            link.remove(collision)
            print(f"Removed collision from: {link.get('name')}")

# 4. Save the file
with open(output_file, 'wb') as f:
    tree.write(f, encoding='utf-8')

# Strip the <?xml... declaration that crashes ROS 2
with open(output_file, 'r') as f:
    lines = f.readlines()
with open(output_file, 'w') as f:
    for line in lines:
        if not line.strip().startswith('<?xml'):
            f.write(line)

print("Done! Collisions removed and file is ready for Gazebo.")