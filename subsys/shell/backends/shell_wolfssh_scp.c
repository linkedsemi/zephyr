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
 * Downloads are the mirror image: the client's source path is reduced to its
 * last path element and served from SHELL_SCP_DIR, so the same traversal
 * argument applies. Only single-file downloads are supported, and the same
 * size cap as for uploads is enforced. Documented usage:
 *
 *   scp -O   <user>@<bmc>:/mnt/tmp/<name>  <local>
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

/* Download side state: one file is served per "scp -f" session. The handle is
 * released as soon as the last byte has been handed out and again at the end
 * of the session, so an aborted transfer cannot leak it. */
static struct fs_file_t scp_dl_file;
static bool scp_dl_open;
static bool scp_dl_started;	/* metadata for this request already reported */
static bool scp_dl_done;	/* every byte of this file has been handed out */
static word32 scp_dl_total;	/* size reported to the peer for this file */
static char scp_dl_path[sizeof(SHELL_SCP_DIR) + 160];

static void scp_dl_close(void)
{
	if (scp_dl_open) {
		fs_close(&scp_dl_file);
		scp_dl_open = false;
	}
}

/* Report "msg" to the peer (scp prints it) and tear the transfer down. */
static int scp_dl_abort(WOLFSSH *ssh, const char *msg)
{
	scp_dl_close();
	scp_dl_started = false;	/* an aborted request is not a completed one */
	wolfSSH_SetScpErrorMsg(ssh, msg);
	return WS_SCP_ABORT;
}

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

/* Streams one file out. Called once per chunk with fileOffset telling where
 * to resume; returns the number of bytes copied into buf, or 0 once the whole
 * file has been handed out -- the library then finishes the transfer itself
 * (it compares scpFileOffset against scpFileSz and sends the trailing
 * confirmation byte), so WS_SCP_COMPLETE must NOT be returned here: it would
 * skip that confirmation and the client would report a failed transfer. */
static int scp_send_file(WOLFSSH *ssh, const char *peerRequest,
			 char *fileName, word32 fileNameSz, word64 *mTime,
			 word64 *aTime, int *fileMode, word32 fileOffset,
			 word32 *totalFileSz, byte *buf, word32 bufSz)
{
	struct fs_dirent probe;
	const char *name;
	ssize_t r;
	int len;

	if (!scp_dl_started) {
		/* First call for this request. The client's requested source
		 * path arrives in peerRequest (ssh->scpBasePath, already run
		 * through wolfSSH_CleanPath()); unlike the upload direction,
		 * ParseScpCommand() does NOT split it for "scp -f" -- it only
		 * calls ScpCheckForRename() for "scp -t" -- so fileName is
		 * still the empty buffer ScpSourceInit() just allocated and we
		 * have to derive the name ourselves. Like the upload side, the
		 * directory part is ignored and the file is served from
		 * SHELL_SCP_DIR, which is what keeps path traversal out of the
		 * picture. */
		name = scp_basename(peerRequest);
		if (name == NULL) {
			return scp_dl_abort(ssh, "no file name given");
		}
		if (strpbrk(name, "*?") != NULL) {
			return scp_dl_abort(ssh, "wildcard download not "
				"supported, fetch files individually");
		}

		len = snprintk(scp_dl_path, sizeof(scp_dl_path), "%s/%s",
			       SHELL_SCP_DIR, name);
		if (len < 0 || len >= (int)sizeof(scp_dl_path)) {
			return scp_dl_abort(ssh, "file name too long");
		}

		if (fs_stat(scp_dl_path, &probe) < 0) {
			return scp_dl_abort(ssh, "no such file");
		}
		if (probe.type != FS_DIR_ENTRY_FILE) {
			return scp_dl_abort(ssh, "not a regular file");
		}
		if (probe.size > SHELL_SCP_MAX_SZ) {
			return scp_dl_abort(ssh, "file too large");
		}

		fs_file_t_init(&scp_dl_file);
		if (fs_open(&scp_dl_file, scp_dl_path, FS_O_READ) < 0) {
			return scp_dl_abort(ssh, "unable to open file");
		}

		scp_dl_total = (word32)probe.size;
		scp_dl_open = true;
		scp_dl_started = true;

		/* Metadata for the peer's "T" and "C" headers. Zephyr's fs
		 * does not expose timestamps, so 0 is reported and the peer
		 * stamps its local copy with its own clock. fileName is what
		 * the client saves the copy as. */
		*mTime = 0;
		*aTime = 0;
		*fileMode = 0644;
		*totalFileSz = scp_dl_total;
		if (strlen(name) + 1 > fileNameSz) {
			return scp_dl_abort(ssh, "file name too long");
		}
		strcpy(fileName, name);

		printk("scp: serving %s (%u bytes)\n", scp_dl_path,
		       scp_dl_total);
	}

	if (!scp_dl_open || fileOffset >= scp_dl_total) {
		scp_dl_close();
		scp_dl_done = true;
		return 0;
	}

	if (fs_seek(&scp_dl_file, (off_t)fileOffset, FS_SEEK_SET) < 0) {
		return scp_dl_abort(ssh, "unable to seek in file");
	}

	r = fs_read(&scp_dl_file, buf, bufSz);
	if (r < 0) {
		return scp_dl_abort(ssh, "unable to read file");
	}
	if (r == 0) {
		/* real size shorter than the stat size: stop here */
		scp_dl_close();
		return 0;
	}

	if ((word32)fileOffset + (word32)r >= scp_dl_total) {
		scp_dl_close();		/* last chunk: release the handle now */
		scp_dl_done = true;	/* every byte has been handed out */
	}

	return (int)r;
}

/* Source side: the client pulls a file from us ("scp -f"). Only single-file
 * downloads from SHELL_SCP_DIR are served, mirroring the upload policy. */
static int scp_send_cb(WOLFSSH *ssh, int state, const char *peerRequest,
		       char *fileName, word32 fileNameSz, word64 *mTime,
		       word64 *aTime, int *fileMode, word32 fileOffset,
		       word32 *totalFileSz, byte *buf, word32 bufSz, void *ctx)
{
	ARG_UNUSED(ctx);

	/* wolfSSH calls this once at SCP_SOURCE_BEGIN with every output pointer
	 * NULL and state == WOLFSSH_SCP_NEW_REQUEST, and then once per chunk
	 * from SCP_TRANSFER. The state there is ssh->scpRequestType, which is
	 * *not* initialised per session (SshInit() zeroes the whole struct and
	 * only "scp -r" ever writes it), so a single-file download also arrives
	 * with WOLFSSH_SCP_NEW_REQUEST. Distinguish the two calls by the output
	 * pointers rather than by the state. */
	if (totalFileSz == NULL) {
		scp_dl_close();
		scp_dl_started = false;
		scp_dl_done = false;
		return WS_SUCCESS;
	}

	switch (state) {
	case WOLFSSH_SCP_SINGLE_FILE_REQUEST:
	case WOLFSSH_SCP_CONTINUE_FILE_TRANSFER:
	case WOLFSSH_SCP_NEW_REQUEST:	/* zero-init: single file, 1st chunk */
		return scp_send_file(ssh, peerRequest, fileName, fileNameSz,
				     mTime, aTime, fileMode, fileOffset,
				     totalFileSz, buf, bufSz);

	case WOLFSSH_SCP_RECURSIVE_REQUEST:
	case WOLFSSH_SCP_NEW_DIR:
	case WOLFSSH_SCP_END_DIR:
		return scp_dl_abort(ssh, "recursive download not supported");

	default:
		return scp_dl_abort(ssh, "unsupported scp request");
	}
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

	/* Once the last file was written and closed (upload) or fully handed
	 * out (download), a failing accept() only means "the peer hung up":
	 * scp exits right after the transfer and closes the TCP connection.
	 * For an upload that surfaces as WS_EOF / WS_CHANNEL_CLOSED while we
	 * wait for its CHANNEL_CLOSE; for a download the server's own final
	 * CHANNEL_CLOSE (wolfSSH_stream_exit() at the end of DoScpSource())
	 * writes into a socket the client has already closed, which reliably
	 * surfaces as WS_SOCKET_ERROR_E. Both are the success they are, so a
	 * completed transfer does not look like a failure on the console. */
	if (ret != WS_SCP_COMPLETE && (scp_file_done || scp_dl_done) &&
	    (err == WS_EOF || err == WS_CHANNEL_CLOSED ||
	     err == WS_SOCKET_ERROR_E)) {
		ret = WS_SCP_COMPLETE;
	}

	/* A transfer that was aborted mid-way would otherwise leave the file
	 * open until the next session. */
	scp_dl_close();
	scp_dl_started = false;
	scp_dl_done = false;

	return ret;
}
