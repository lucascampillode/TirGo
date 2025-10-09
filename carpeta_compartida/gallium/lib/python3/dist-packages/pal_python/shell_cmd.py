#!/usr/bin/env python

import subprocess
import tempfile
import time
import os
import signal

from builtins import object

class ShellCmd(object):

    def __init__(self, cmd, stdin=None, stdout=None, stderr=None, shell=True):
        self.cmd_stdin = stdin
        self.cmd_stdout = stdout
        self.cmd_stderr = stderr

        if self.cmd_stdin:
            self.inf = self.cmd_stdin
        else:
            self.inf = tempfile.NamedTemporaryFile(mode="r")

        if self.cmd_stdout:
            self.outf = self.cmd_stdout
        else:
            self.outf = tempfile.NamedTemporaryFile(mode="w")

        if self.cmd_stderr:
            self.errf = self.cmd_stderr
        else:
            self.errf = tempfile.NamedTemporaryFile(mode="w")

        self.process = subprocess.Popen(cmd, shell=shell, stdin=self.inf,
                                        stdout=self.outf, stderr=self.errf,
                                        preexec_fn=os.setsid)
        self.old_stdout_len = 1
        self.old_stderr_len = 1

    def __del__(self):
        if not self.is_done():
            self.kill()

        # close only the fds created by ourselves
        if self.cmd_stdin is None:
            self.inf.close()
        if self.cmd_stdout is None:
            self.outf.close()
        if self.cmd_stderr is None:
            self.errf.close()

    def get_stdout(self):
        with open(self.outf.name, "r") as f:
            return f.read()

    def get_stderr(self):
        with open(self.errf.name, "r") as f:
            return f.read()

    def get_updated_stdout(self):
        read_output = self.get_stdout().split('\n')
        output, self.old_stdout_len = self.compute_diff_content(
            read_output, self.old_stdout_len)
        return output

    def get_updated_stderr(self):
        read_output = self.get_stderr().split('\n')
        output, self.old_stderr_len = self.compute_diff_content(
            read_output, self.old_stderr_len)
        return output

    def compute_diff_content(self, updated_content, old_content_length):
        content = len(updated_content) - old_content_length
        if content > 0:
            output = "\n".join(updated_content[old_content_length - 1:])
            return output, len(updated_content)
        else:
            return None, len(updated_content)

    def stream_stdout(self):
        old_content_length = 1
        while not self.is_done():
            read_output = self.get_stdout().split('\n')
            diff_content, old_content_length = self.compute_diff_content(
                read_output, old_content_length)
            yield diff_content
        # Helps to captute the remaining content after the process is completed
        read_output = self.get_stdout().split('\n')
        diff_content, old_content_length = self.compute_diff_content(
            read_output, old_content_length)
        yield diff_content

    def stream_stderr(self):
        old_content_length = 1
        while not self.is_done():
            read_output = self.get_stderr().split('\n')
            diff_content, old_content_length = self.compute_diff_content(
                read_output, old_content_length)
            yield diff_content
        # Helps to captute the remaining content after the process is completed
        read_output = self.get_stdout().split('\n')
        diff_content, old_content_length = self.compute_diff_content(
            read_output, old_content_length)
        yield diff_content

    def get_retcode(self):
        """get retcode or None if still running"""
        return self.process.poll()

    def is_done(self):
        return self.process.poll() is not None

    def wait(self, timeout=float("inf")):
        """wait for the process to end"""
        start_time = time.clock()
        elapsed = 0.0
        while elapsed < timeout:
            if self.is_done():
                return True
            time.sleep(0.1)
            elapsed = time.clock() - start_time
        return False

    def kill(self):
        os.killpg(self.process.pid, signal.SIGTERM)
        self.process.wait()

    def nice_kill(self, retry_time=2, max_retries=2):
        """
         Attempts to kill with SIGINT, returns if successful
        """
        retries = 0
        while (not self.is_done() and retries < max_retries):
            if retries > 0:
                time.sleep(retry_time)
                if self.is_done():
                    break  # In case command finished during sleep (ie rosbag)
            os.killpg(self.process.pid, signal.SIGINT)
            retries += 1
        return self.is_done()
