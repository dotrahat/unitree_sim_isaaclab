from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
import yaml

# Load YAML config
with open("config.yaml", "r") as f:
    cfg_dict = yaml.safe_load(f)

# Convert dict → Isaac config object
cfg = UrdfConverterCfg(**cfg_dict)

# Run converter
converter = UrdfConverter(cfg)
converter.convert()

print("✅ Conversion complete")