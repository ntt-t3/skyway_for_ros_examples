const Peer = window.Peer;

(async function main() {
  const localVideo = document.getElementById("js-local-stream");
  const localId = document.getElementById("js-local-id");
  const callTrigger = document.getElementById("js-call-trigger");
  const closeTrigger = document.getElementById("js-close-trigger");
  const remoteVideo = document.getElementById("js-remote-stream");
  const remoteId = document.getElementById("js-remote-id");
  const meta = document.getElementById("js-meta");
  const sdkSrc = document.querySelector("script[src*=skyway]");

  meta.innerText = `
    UA: ${navigator.userAgent}
    SDK: ${sdkSrc ? sdkSrc.src : "unknown"}
  `.trim();

  const localStream = await navigator.mediaDevices
    .getUserMedia({
      audio: true,
      video: true,
    })
    .catch(console.error);

  // Render local stream
  localVideo.muted = true;
  localVideo.srcObject = localStream;
  localVideo.playsInline = true;
  await localVideo.play().catch(console.error);

  let api_key = window.__SKYWAY_KEY__;

  const queryString = window.location.search;
  const urlParams = new URLSearchParams(queryString);
  const key = urlParams.get("api_key");
  if (key) api_key = key;

  console.log(api_key);
  // peer_idはnew Peerの第一引数で指定しなければランダムで割り当てられるが、
  // 毎回ROS側で変更が手間であればここで設定する。
  const peer_id = "media";
  const peer = (window.peer = new Peer(peer_id, {
    key: api_key,
    debug: 3,
  }));

  // Register caller handler
  callTrigger.addEventListener("click", () => {
    // Note that you need to ensure the peer has connected to signaling server
    // before using methods of peer instance.
    if (!peer.open) {
      return;
    }

    const mediaConnection = peer.call(remoteId.value, localStream);

    mediaConnection.on("stream", async (stream) => {
      // Render remote stream for caller
      remoteVideo.srcObject = stream;
      remoteVideo.playsInline = true;
      await remoteVideo.play().catch(console.error);
    });

    mediaConnection.once("close", () => {
      remoteVideo.srcObject.getTracks().forEach((track) => track.stop());
      remoteVideo.srcObject = null;
    });

    closeTrigger.addEventListener("click", () => mediaConnection.close(true));
  });

  peer.once("open", (id) => (localId.textContent = id));

  // Register callee handler
  peer.on("call", (mediaConnection) => {
    mediaConnection.answer(localStream);

    mediaConnection.on("stream", async (stream) => {
      // Render remote stream for callee
      remoteVideo.srcObject = stream;
      remoteVideo.playsInline = true;
      await remoteVideo.play().catch(console.error);
    });

    mediaConnection.once("close", () => {
      remoteVideo.srcObject.getTracks().forEach((track) => track.stop());
      remoteVideo.srcObject = null;
    });

    closeTrigger.addEventListener("click", () => mediaConnection.close(true));
  });

  peer.on("error", console.error);
})();
