// Demo players load on click: each recording is megabytes, and browsers cap WebGL contexts.
function loadPlayer(button) {
  const frame = document.createElement("iframe");
  frame.src = button.dataset.src;
  frame.title = button.textContent;
  frame.allow = "fullscreen";
  frame.className = "demo-player";
  button.replaceWith(frame);
}

document.addEventListener("click", (event) => {
  const button = event.target.closest("button.demo-player[data-src]");
  if (button !== null) loadPlayer(button);
});

// `#play` starts the page's first player: the README's poster links the home page with it,
// so one click there lands on a playing demo.
if (location.hash === "#play") {
  const button = document.querySelector("button.demo-player[data-src]");
  if (button !== null) loadPlayer(button);
}
