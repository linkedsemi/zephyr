/* shell_wolfssh_scp.c
 *
 * Minimal SCP server support (upload only) for the wolfSSH shell backend.
 *
 * Integration is deliberately tiny: wolfSSH_accept() returns WS_SCP_INIT once
 * the client's "exec" request carrying an "scp ..." command has been handled
 * (ssh.c: ChannelCommandIsScp() -> ACCEPT_INIT_SCP_TRANSFER), and returns
 * WS_SCP_COMPLETE after the transfer loop (DoScpRequest()) has run to the
 * end. shell_wolfssh.c therefore only has to recognize WS_SCP_INIT and call
 * shell_wolfssh_scp_accept() here.
 *
 * This TU defers WOLFSSH_ZEPHYR itself -- on purpose. Only sources in the
 * "app" CMake target get it (the wolfSSH module does
 * target_compile_definitions(app PUBLIC WOLFSSH_ZEPHYR)); without it
 * wolfssh/settings.h does not include CONFIG_WOLFSSH_SETTINGS_FILE, so
 * WOLFSSH_SCP -- and with it the wolfSSH_SetScpRecv/Send() declarations --
 * would not be visible.
 *
 * Uploaded files always land in SHELL_SCP_DIR "/" basename(fileName). The
 * remote directory requested by the client is intentionally ignored: it keeps
 * path traversal out of the picture and side-steps wolfSSH's
 * ScpCheckForRename() semantics (it rewrites the target path and drops the
 * file name sent in the protocol's "C" message whenever the requested path
 * does not end in '/'). Documented usage is therefore:
 *
 *   scp -O   <local>  <user>@<bmc>:/mnt/tmp/
 *   scp -O   <local>  <user>@<bmc>:/mnt/tmp/<name>
 *
 * "-O" is required with OpenSSH >= 9.0, whose scp defaults to the SFTP
 * protocol; WinSCP defaults to SFTP as well, pscp uses the legacy protocol.
 */

#ifndef WOLFSSH_ZEPHYR
#define WOLFSSH_ZEPHYR
#endif

#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/fs/fs.h>
#include <zephyr/net/socket.h>

#include <wolfssh/wolfscp.h>

/* Drop directory for uploaded files (created on demand) and the largest file
 * accepted. Both are policy for this debug/config upload path only. */
#define SHELL_SCP_DIR    "/mnt/tmp"
#define SHELL_SCP_MAX_SZ (8U * 1024U * 1024U)

static struct fs_file_t scp_file;
static bool scp_file_open;
/* Set once the last file of this session was written and closed. Used to tell
 * "transfer finished, the peer just hung up" apart from a real abort. */
static bool scp_file_done;
static char scp_path[sizeof(SHELL_SCP_DIR) + 160];

/* Last path component of an SCP file name, or NULL when there is nothing
 * usable in it. Never returns an empty name, "." or ".." so the result is
 * safe to append to SHELL_SCP_DIR. */
static const char *scp_basename(const char *name)
{
	const char *s;

	if (name == NULL) {
		return NULL;
	}

	s = strrchr(name, '/');
	s = (s != NULL) ? (s + 1) : name;

	if (*s == '\0' || strcmp(s, ".") == 0 || strcmp(s, "..") == 0) {
		return NULL;
	}

	return s;
}

/* Sink side: the client sends a file to us ("scp -t"). */
static int scp_recv_cb(WOLFSSH *ssh, int state, const char *basePath,
		       const char *fileName, int fileMode, word64 mTime,
		       word64 aTime, word32 totalFileSz, byte *buf,
		       word32 bufSz, word32 fileOffset, void *ctx)
{
	const char *name;
	int len;

	ARG_UNUSED(fileMode);
	ARG_UNUSED(mTime);
	ARG_UNUSED(aTime);
	ARG_UNUSED(fileOffset);
	ARG_UNUSED(ctx);

	switch (state) {
	case WOLFSSH_SCP_NEW_REQUEST:
		/* Directory must exist before the first file is created; the
		 * mount point itself (/mnt) is already there. */
		scp_file_done = false;
		len = fs_mkdir(SHELL_SCP_DIR);
		if (len < 0 && len != -EEXIST) {
			wolfSSH_SetScpErrorMsg(ssh, "cannot create upload dir");
			return WS_SCP_ABORT;
		}
		return WS_SCP_CONTINUE;

	case WOLFSSH_SCP_NEW_FILE: {
		struct fs_dirent probe;

		name = scp_basename(fileName);
		if (name == NULL) {
			wolfSSH_SetScpErrorMsg(ssh, "invalid file name");
			return WS_SCP_ABORT;
		}
		if (totalFileSz > SHELL_SCP_MAX_SZ) {
			wolfSSH_SetScpErrorMsg(ssh, "file too large");
			return WS_SCP_ABORT;
		}

		/* A destination without a trailing '/' is rewritten by wolfSSH
		 * (ScpCheckForRename()): the last path element becomes
		 * fileName and basePath is truncated to its parent -- and the
		 * file name from the protocol's "C" message is dropped. If that
		 * rewritten name is an existing *directory*, the client meant
		 * "copy into this directory" and we can no longer learn the
		 * file name it intended: refuse with an actionable message
		 * instead of silently creating a file called after the
		 * directory. */
		if (basePath != NULL) {
			char probe_path[sizeof(scp_path)];
			int n = snprintk(probe_path, sizeof(probe_path),
					 "%s/%s", basePath, fileName);

			if (n > 0 && n < (int)sizeof(probe_path) &&
			    fs_stat(probe_path, &probe) == 0 &&
			    probe.type == FS_DIR_ENTRY_DIR) {
				wolfSSH_SetScpErrorMsg(ssh, "target is a "
					"directory, append '/' to the path");
				return WS_SCP_ABORT;
			}
		}

		len = snprintk(scp_path, sizeof(scp_path), "%s/%s",
			       SHELL_SCP_DIR, name);
		if (len < 0 || len >= (int)sizeof(scp_path)) {
			wolfSSH_SetScpErrorMsg(ssh, "file name too long");
			return WS_SCP_ABORT;
		}

		fs_file_t_init(&scp_file);
		if (fs_open(&scp_file, scp_path,
			    FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC) < 0) {
			wolfSSH_SetScpErrorMsg(ssh, "unable to open file");
			return WS_SCP_ABORT;
		}
		scp_file_open = true;
		return WS_SCP_CONTINUE;
	}

	case WOLFSSH_SCP_FILE_PART:
		if (!scp_file_open ||
		    fs_write(&scp_file, buf, bufSz) != (ssize_t)bufSz) {
			fs_close(&scp_file);
			scp_file_open = false;
			wolfSSH_SetScpErrorMsg(ssh, "unable to write file");
			return WS_SCP_ABORT;
		}
		return WS_SCP_CONTINUE;

	case WOLFSSH_SCP_FILE_DONE:
		if (scp_file_open) {
			fs_sync(&scp_file);
			fs_close(&scp_file);
			scp_file_open = false;
			scp_file_done = true;
			printk("scp: %s (%u bytes) saved\n", scp_path,
			       totalFileSz);
		}
		return WS_SCP_CONTINUE;

	case WOLFSSH_SCP_NEW_DIR:
	case WOLFSSH_SCP_END_DIR:
		wolfSSH_SetScpErrorMsg(ssh, "recursive upload not supported");
		return WS_SCP_ABORT;

	default:
		wolfSSH_SetScpErrorMsg(ssh, "unsupported scp request");
		return WS_SCP_ABORT;
	}
}

/* Source side: the client pulls a file from us ("scp -f"). Not implemented
 * yet; the callback must still exist because wolfSSH calls it
 * unconditionally once an scp transfer is started (DoScpSource()). */
static int scp_send_cb(WOLFSSH *ssh, int state, const char *peerRequest,
		       char *fileName, word32 fileNameSz, word64 *mTime,
		       word64 *aTime, int *fileMode, word32 fileOffset,
		       word32 *totalFileSz, byte *buf, word32 bufSz, void *ctx)
{
	ARG_UNUSED(peerRequest);
	ARG_UNUSED(fileName);
	ARG_UNUSED(fileNameSz);
	ARG_UNUSED(mTime);
	ARG_UNUSED(aTime);
	ARG_UNUSED(fileMode);
	ARG_UNUSED(fileOffset);
	ARG_UNUSED(totalFileSz);
	ARG_UNUSED(buf);
	ARG_UNUSED(bufSz);
	ARG_UNUSED(ctx);

	if (state == WOLFSSH_SCP_NEW_REQUEST) {
		return WS_SUCCESS;
	}

	wolfSSH_SetScpErrorMsg(ssh, "download not supported");
	return WS_SCP_ABORT;
}

/* Called once per WOLFSSH_CTX (i.e. before any session is accepted). */
void shell_wolfssh_scp_register_ctx(WOLFSSH_CTX *ctx)
{
	wolfSSH_SetScpRecv(ctx, scp_recv_cb);
	wolfSSH_SetScpSend(ctx, scp_send_cb);
}

/* True when the SCP command asks for a target at the filesystem root, e.g.
 * "scp -t /", "scp -r -t ///".
 *
 * Such a target must be refused before entering the library state machine:
 * DoScpRequest() -> ParseScpCommand() -> ScpCheckForRename() builds
 * "<target>/.." and runs it through wolfSSH_CleanPath(), whose ".." removal
 * loop starts scanning at index 1 (internal.c:15753). For a root target that
 * loop never sees the '/' in front of "..": the buffer is "/..", i=1 matches,
 * enIdx = i + 3 = 4 while sz = 3, and the cleanup then does
 *
 *     WMEMMOVE(path + prIdx, path + enIdx, sz - enIdx)
 *
 * with "sz - enIdx" evaluating to -1, i.e. a memmove of (size_t)-1 bytes --
 * the session dies with the heap
 * scribbled over (observed as "lost connection" on the client). A target like
 * "/mnt/tmp/" is safe because there enIdx == sz (size 0). */
static bool scp_target_is_root(const char *cmd)
{
	const char *p;

	if (cmd == NULL) {
		return false;
	}

	p = strstr(cmd, " -t");
	if (p == NULL) {
		return false;		/* "scp -f ..." belongs to the sender side */
	}

	p += 3;
	while (*p == ' ') {
		p++;
	}
	if (*p == '\0') {
		return false;		/* empty target: rejected by the library */
	}

	while (*p == '/') {
		p++;
	}

	return (*p == '\0');
}

/* Drives an SCP session started by wolfSSH_accept() == WS_SCP_INIT up to
 * WS_SCP_COMPLETE. The socket is non-blocking, so WS_WANT_READ/WRITE just
 * means "wait for the socket, then call wolfSSH_accept() again"; the poll
 * timeout bounds how long a stalled client can hold this thread. */
int shell_wolfssh_scp_accept(WOLFSSH *ssh)
{
	int ret;
	int err;

	if (scp_target_is_root(wolfSSH_GetSessionCommand(ssh))) {
		/* Answer with the protocol's "fatal error" byte followed by a
		 * printable message (same framing as wolfSSH's own
		 * SendScpConfirmation()), so scp prints it instead of just
		 * reporting a dropped connection. */
		static const char rootErr[] = "\x02scp: root is not a valid "
			"target, use e.g. /mnt/tmp/\n";

		printk("scp: refused root target (wolfSSH CleanPath bug)\n");
		(void)wolfSSH_stream_send(ssh, (byte *)rootErr,
					  sizeof(rootErr));
		return WS_SCP_COMPLETE;
	}

	do {
		ret = wolfSSH_accept(ssh);
		err = wolfSSH_get_error(ssh);

		if (ret == WS_SUCCESS || ret == WS_SCP_COMPLETE) {
			break;
		}
		if (err != WS_WANT_READ && err != WS_WANT_WRITE) {
			break;
		}

		struct pollfd pfd = {
			.fd = wolfSSH_get_fd(ssh),
			.events = (err == WS_WANT_WRITE) ? POLLOUT : POLLIN,
		};

		if (poll(&pfd, 1, CONFIG_SHELL_WOLFSSH_TIMEOUT * 1000) <= 0) {
			printk("scp: socket wait timed out\n");
			break;
		}
	} while (1);

	/* Once the last file was written and closed, a failing accept() only
	 * means "the peer hung up while we were waiting for its CHANNEL_CLOSE"
	 * (scp exits right after the transfer and closes the TCP connection,
	 * which surfaces as WS_FATAL_ERROR with ssh->error == WS_EOF; only
	 * WS_SOCKET_ERROR_E / WS_CHANNEL_CLOSED are treated as "peer hung up,
	 * but SCP is done" inside DoScpRequest()). Report that as the success
	 * it is, so a completed upload does not look like a failure. */
	if (ret != WS_SCP_COMPLETE && scp_file_done &&
	    (err == WS_EOF || err == WS_CHANNEL_CLOSED)) {
		ret = WS_SCP_COMPLETE;
	}

	return ret;
}
