import os
import re
import base64


def change_to_build(og_path: str) -> str:
    d = og_path.split(os.sep)
    d[-2] = "bsvg"
    p = os.sep.join(d)
    return p


root_dir = os.getcwd()
conf_dir = os.path.join(".github", "conf")
build_dir = os.path.join(".github", "bsvg")
src_dir = os.path.join(".github", "svg")
template_file = os.path.join(conf_dir, "template.svg")
preset_file = os.path.join(conf_dir, "presets.txt")

print(
    f"Root dir: {root_dir}\nConf dir: {conf_dir}\nSVG sources dir: {src_dir}\nBuild dir: {build_dir}\nTemplate file: {template_file}\n"
)

# Read presets
if not os.path.exists(preset_file):
    raise FileNotFoundError(f"File {preset_file} not found!")
presets = []
with open(preset_file, "r") as preset_file:
    presets = [tuple(l.split(";")) for l in preset_file.readlines()]
print(
    "Presets:\n\t"
    + "\n\t".join([f"`{p[1]}` ({p[0]}.svg) with image `{p[2]}`" for p in presets])
)

# Make sources
if len(presets) > 0:
    # Read template
    if not os.path.exists(template_file):
        raise FileNotFoundError(f"File {template_file} not found!")
    template = ""
    with open(template_file, "r") as tfile:
        template = tfile.read()

    if not os.path.exists(src_dir):
        os.mkdir(src_dir)

    # Make sources
    for file_name, robot_name, robot_img in presets:
        with open(os.path.join(src_dir, f"{file_name}.svg"), "w+") as src_file:
            src_file.write(
                template.replace("${name}", robot_name).replace(
                    "${img_path}", robot_img
                )
            )


# Get files to update
files = []
for dirpath, _, filenames in os.walk(src_dir):
    for f in filenames:
        if f.endswith(".svg"):
            files.append(os.path.join(dirpath, f))

print("Found source files:\n\t" + "\n\t".join(files))

# Replace images in svg
if not os.path.exists(build_dir):
    os.mkdir(build_dir)
for f in files:
    assert type(f) is str
    built_file = change_to_build(f)
    print(f"Processing file {f} -> {built_file}")
    with open(f, "r") as src:
        with open(built_file, "w+") as dest:
            file_content = src.read()

            # Find the images tags
            found = re.findall('<img[\w "=\/]+src\="([\w\/\.]+)', file_content)
            done = []
            for path in found:
                if path in done:
                    continue
                print(f"\tNeed image at path `{path}`... ", end="")
                if not os.path.exists(path):
                    print("Doesn't exist!")
                    continue
                with open(os.path.join(root_dir, path), "rb") as img_file:
                    img_data = base64.b64encode(img_file.read())
                    file_content = file_content.replace(
                        path, f"data:image/png;charset=utf-8;base64,{img_data.decode()}"
                    )
                print("OK")
                done.append(path)
            dest.write(file_content)
