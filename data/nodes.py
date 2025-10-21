import csv

input_csv = "nodes.csv"
output_lua = "nodes.lua"

lua_lines = ["local node_positions = {"]

with open(input_csv, newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        x = row['x']
        y = row['y']
        lua_lines.append(f"\t{{ x = {x}, y = {y} }},")
        
lua_lines.append("}")

with open(output_lua, "w") as f:
    f.write("\n".join(lua_lines))

print(f"Converted {input_csv} → {output_lua}")