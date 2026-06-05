#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_PATH="$SCRIPT_DIR/../../install/scuttle_description/share/scuttle_description"
MODELS_PATH="$HOME/.gz/models/scuttle_description"

mkdir -p "$MODELS_PATH"

if [ -d "$INSTALL_PATH/meshes" ]; then
    cp -r "$INSTALL_PATH/meshes" "$MODELS_PATH/" 2>/dev/null
    echo "✅ Meshes copied to: $MODELS_PATH"
fi

cat > "$MODELS_PATH/model.config" << 'EOF'
<?xml version="1.0"?>
<model>
  <n>scuttle_description</n>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><n>SCUTTLE</n></author>
  <description>SCUTTLE Robot</description>
</model>
EOF

cat > "$MODELS_PATH/model.sdf" << 'EOF'
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="scuttle_description">
    <static>false</static>
    <link name="base_link"/>
  </model>
</sdf>
EOF
