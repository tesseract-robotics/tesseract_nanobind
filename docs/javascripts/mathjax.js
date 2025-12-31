// MathJax configuration for tesseract_robotics documentation
window.MathJax = {
  tex: {
    inlineMath: [["\\(", "\\)"]],
    displayMath: [["\\[", "\\]"]],
    processEscapes: true,
    processEnvironments: true,
    tags: 'ams'
  },
  options: {
    ignoreHtmlClass: ".*|",
    processHtmlClass: "arithmatex"
  },
  loader: {
    load: ['[tex]/ams']
  }
};

// Document ready handler for any custom initialization
document$.subscribe(function() {
  // Re-render MathJax when content changes (e.g., tabs)
  if (typeof MathJax !== 'undefined') {
    MathJax.typesetPromise();
  }
});
