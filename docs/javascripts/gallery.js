// Gallery players load on click: each recording is megabytes, and browsers cap WebGL contexts.
document.addEventListener("click", (event) => {
  const button = event.target.closest("button.gallery-player[data-src]");
  if (button === null) return;
  const frame = document.createElement("iframe");
  frame.src = button.dataset.src;
  frame.title = button.textContent;
  frame.allow = "fullscreen";
  frame.className = "gallery-player";
  button.replaceWith(frame);
});
