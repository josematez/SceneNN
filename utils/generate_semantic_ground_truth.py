import xml.etree.ElementTree as ET
import json
import argparse


def process_gt_xml(xml_file: str):
    """
    Parse the XML file and process each label with a semantic text.
    Returns a dictionary ready to be saved as JSON.
    """
    tree = ET.parse(xml_file)
    root = tree.getroot()

    instances = {}
    obj_index = 0

    # Iterate over all label elements
    for label in root.findall('label'):
        text = label.get('text')
        # Process only if semantic label exists and is not empty
        if text is not None and text.strip() != "":
            aabb_str = label.get('aabbox')
            if aabb_str is None:
                continue

            # Expecting 6 values: min_x, min_y, min_z, max_x, max_y, max_z
            parts = aabb_str.split()
            if len(parts) != 6:
                print(
                    f"Warning: label {label.get('id')} does not have 6 values in 'aabb'")
                continue

            values = list(map(float, parts))
            min_x, min_y, min_z, max_x, max_y, max_z = values

            # Compute center and size based on min and max coordinates
            center = [
                (min_x + max_x) / 2,
                (min_y + max_y) / 2,
                (min_z + max_z) / 2
            ]
            size = [
                max_x - min_x,
                max_y - min_y,
                max_z - min_z
            ]

            # Change axis order from (x, y, z) to (-x, -z, y) for Robotics standard
            center = [center[0], -center[2], center[1]]
            size = [size[0], size[2], size[1]]

            # Build the instance entry
            instance_key = f"obj{obj_index}"
            instances[instance_key] = {
                "bbox": {
                    "center": center,
                    "size": size
                },
                "results": {
                    text: 1.0
                }
            }
            obj_index += 1

    return {"instances": instances}


def main(args):

    data = process_gt_xml(args.xml_file)

    # Save output JSON
    with open(args.output_path, "w") as f:
        json.dump(data, f, indent=4)
    print(f"Output saved to {args.output_path}")


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="Convert AABB in an XML file to a semantic map in the Voxeland style and save as JSON.")
    parser.add_argument("xml_file",
                        help="Path to the semantic ground truth .xml file")
    parser.add_argument("-o", "--output-path",
                        type=str,
                        default="semantic_ground_truth.json",
                        help="Output file name (default: semantic_ground_truth.json)")

    args = parser.parse_args()

    main(args)
