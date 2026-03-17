#!/bin/bash
# update-version.sh — generate latest.version.json after idf.py build
# Run from the project root (where .git lives)
#
# Usage:
#   ./update-version.sh                    # just generate the file
#   ./update-version.sh --deploy           # generate and scp to server
#

VERSION=$(git describe --always --dirty)
DATE=$(date +%Y-%m-%d)

cat > latest.version.json <<EOF
{"version":"${VERSION}","date":"${DATE}"}
EOF

echo "latest.version.json: ${VERSION} (${DATE})"

if [ "$1" = "--deploy" ]; then
    scp latest.version.json adsb-scope.offx1.com:/var/www/adsb-scope/latest.version.json
    echo "Deployed to server"
fi
