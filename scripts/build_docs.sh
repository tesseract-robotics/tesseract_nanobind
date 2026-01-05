#!/bin/bash
# Build documentation using MkDocs
#
# Usage:
#   ./scripts/build_docs.sh          # Build and serve at localhost:8001
#   ./scripts/build_docs.sh serve    # Start dev server with hot reload
#   ./scripts/build_docs.sh deploy   # Deploy to GitHub Pages

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

# Check dependencies
check_deps() {
    local missing=()

    python -c "import mkdocs" 2>/dev/null || missing+=("mkdocs")
    python -c "import material" 2>/dev/null || missing+=("mkdocs-material")
    python -c "import mkdocstrings" 2>/dev/null || missing+=("mkdocstrings[python]")

    if [ ${#missing[@]} -gt 0 ]; then
        echo "Installing missing dependencies: ${missing[*]}"
        pip install "${missing[@]}"
    fi
}

# Build static site and serve
build() {
    echo "Building documentation..."
    check_deps
    mkdocs build
    echo ""
    echo "Documentation built in site/"
    echo "Starting server at http://127.0.0.1:8001"

    # Open browser (works on macOS, Linux with xdg-open, or WSL)
    if command -v open &> /dev/null; then
        open http://127.0.0.1:8001
    elif command -v xdg-open &> /dev/null; then
        xdg-open http://127.0.0.1:8001
    fi

    # Serve the built site
    python -m http.server 8001 --directory site
}

# Start development server
serve() {
    echo "Starting development server..."
    check_deps
    echo "Open http://localhost:8000 in your browser"
    echo "Press Ctrl+C to stop"
    mkdocs serve
}

# Deploy to GitHub Pages
deploy() {
    echo "Deploying to GitHub Pages..."
    check_deps
    mkdocs gh-deploy --force
    echo "Deployed! Visit https://tesseract-robotics.github.io/tesseract_nanobind/"
}

# Clean build artifacts
clean() {
    echo "Cleaning build artifacts..."
    rm -rf site/
    echo "Done"
}

# Main
case "${1:-build}" in
    build)
        build
        ;;
    serve)
        serve
        ;;
    deploy)
        deploy
        ;;
    clean)
        clean
        ;;
    *)
        echo "Usage: $0 {build|serve|deploy|clean}"
        echo ""
        echo "Commands:"
        echo "  build   - Build static site to site/ (default)"
        echo "  serve   - Start development server at localhost:8000"
        echo "  deploy  - Deploy to GitHub Pages"
        echo "  clean   - Remove build artifacts"
        exit 1
        ;;
esac
