"use strict";

const fs = require("fs");
const http = require("http");
const path = require("path");

const host = "127.0.0.1";
const port = 8765;
const root = __dirname;
const mime = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".json": "application/json; charset=utf-8",
};

const server = http.createServer((request, response) => {
  let pathname;
  try {
    pathname = decodeURIComponent(new URL(request.url, `http://${host}:${port}`).pathname);
  } catch (_) {
    response.writeHead(400).end("Bad request");
    return;
  }

  const relative = pathname === "/" ? "bluetooth.html" : pathname.replace(/^\/+/, "");
  const filename = path.resolve(root, relative);
  if (filename !== root && !filename.startsWith(`${root}${path.sep}`)) {
    response.writeHead(403).end("Forbidden");
    return;
  }

  fs.stat(filename, (statError, stat) => {
    if (statError || !stat.isFile()) {
      response.writeHead(404).end("Not found");
      return;
    }
    response.writeHead(200, {
      "Content-Type": mime[path.extname(filename).toLowerCase()] || "application/octet-stream",
      "Cache-Control": "no-store, max-age=0",
      "X-Content-Type-Options": "nosniff",
    });
    fs.createReadStream(filename).pipe(response);
  });
});

server.on("error", (error) => {
  if (error.code === "EADDRINUSE") {
    console.log(`端口 ${host}:${port} 已被占用；通常表示本地服务器已经启动。`);
    console.log(`请直接打开：http://${host}:${port}/official-echo-test.html`);
    process.exitCode = 0;
    return;
  }
  console.error(error);
  process.exitCode = 1;
});

server.listen(port, host, () => {
  console.log(`BalanceBot BLE server: http://${host}:${port}/`);
  console.log(`UART echo test:       http://${host}:${port}/official-echo-test.html`);
});
