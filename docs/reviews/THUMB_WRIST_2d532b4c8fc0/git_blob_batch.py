"""Read immutable Git blobs through one hidden cat-file process.

Only full SHA-1 commit/tree/object ids are accepted. Index paths, branch names,
revision expressions and shell commands are never sent to cat-file. Each tree
is enumerated once; streamed raw bytes are checked against both their Git OID
and SHA-256. This module contains no Git mutation command.
"""
from pathlib import Path, PurePosixPath
import hashlib
import re
import subprocess
import tempfile

OID=re.compile(r'^[0-9a-f]{40}$')


class BatchProtocolError(ValueError):
    pass


def immutable_oid(value):
    if not isinstance(value,str) or not OID.fullmatch(value):
        raise ValueError('Expected a full immutable SHA-1 object id')
    return value


def repository_path(value):
    if not isinstance(value,str) or not value or '\\' in value or ':' in value or any(ord(c)<32 for c in value):
        raise ValueError('Expected a normalized repository-relative path')
    p=PurePosixPath(value)
    if p.is_absolute() or p.as_posix()!=value or not p.parts or any(x in ('..','.git') for x in p.parts):
        raise ValueError('Unsafe repository-relative path')
    return value


def parse_tree_records(raw):
    """Parse binary `ls-tree -rz --full-tree` records without splitting paths."""
    if not isinstance(raw,bytes) or raw and not raw.endswith(b'\0'):
        raise BatchProtocolError('Tree output needs its terminating NUL')
    rows={}
    for record in raw.split(b'\0')[:-1]:
        try:
            meta,path=record.split(b'\t',1)
            mode,kind,oid=meta.decode('ascii').split(' ')
            name=repository_path(path.decode('utf-8'))
            immutable_oid(oid)
        except (ValueError,UnicodeError) as exc:
            raise BatchProtocolError('Invalid tree record') from exc
        if not re.fullmatch(r'[0-7]{6}',mode) or kind not in ('blob','commit','tree') or name in rows:
            raise BatchProtocolError('Invalid/duplicate tree entry')
        rows[name]=dict(mode=mode,type=kind,oid=oid)
    return rows


def read_blob_frame(stream,expected_oid,*,capture=False,max_bytes=None):
    """Read exactly one batch response, including arbitrary binary blob bytes."""
    immutable_oid(expected_oid)
    header=stream.readline(256)
    if not header.endswith(b'\n'):
        raise BatchProtocolError('Missing or overlong batch header')
    fields=header[:-1].split(b' ')
    if len(fields)!=3 or fields[0]!=expected_oid.encode('ascii') or fields[1]!=b'blob' or not fields[2].isdigit():
        raise BatchProtocolError('Missing object, non-blob object or mismatched response id')
    size=int(fields[2])
    if max_bytes is not None and size>max_bytes:
        raise BatchProtocolError('Blob exceeds declared reader limit')
    sha256=hashlib.sha256();git_hash=hashlib.sha1(b'blob '+str(size).encode('ascii')+b'\0')
    chunks=[] if capture else None;remaining=size
    while remaining:
        chunk=stream.read(min(1024*1024,remaining))
        if not chunk or len(chunk)>remaining:
            raise BatchProtocolError('Truncated or invalid blob payload')
        remaining-=len(chunk);sha256.update(chunk);git_hash.update(chunk)
        if chunks is not None:chunks.append(chunk)
    if stream.read(1)!=b'\n':raise BatchProtocolError('Missing blob frame terminator')
    if git_hash.hexdigest()!=expected_oid:raise BatchProtocolError('Raw blob bytes do not match their Git OID')
    return dict(oid=expected_oid,bytes=size,sha256=sha256.hexdigest(),
                data=b''.join(chunks) if chunks is not None else None)


class ImmutableGitBlobs:
    """Single-threaded reader; caller still guards HEAD/index/worktree changes."""
    def __init__(self,git,root,env,temp_directory):
        self.git=str(git);self.root=Path(root).resolve();self.env=dict(env)
        self.env['GIT_NO_REPLACE_OBJECTS']='1'
        self.temp=Path(temp_directory).resolve()
        if self.root.drive.upper()!='D:' or self.temp.drive.upper()!='D:':
            raise ValueError('Repository and temporary directory must remain on D:')
        for key in ('GIT_DIR','GIT_WORK_TREE','GIT_INDEX_FILE'):
            if self.env.get(key):raise ValueError('Refuse inherited '+key)
        self._trees={};self._digests={};self._data={};self._process=None;self._stderr=None;self._closed=False
        self._stats=dict(tree_queries=0,batch_processes=0,blob_requests=0,raw_bytes_read=0,digest_cache_hits=0,byte_cache_hits=0)

    def stats(self):return dict(self._stats)

    def _tree(self,revision):
        immutable_oid(revision)
        if revision not in self._trees:
            p=subprocess.run([self.git,'ls-tree','-rz','--full-tree',revision],cwd=self.root,env=self.env,
                stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,shell=False,
                creationflags=subprocess.CREATE_NO_WINDOW)
            if p.returncode:raise RuntimeError('Immutable tree read failed: '+p.stderr.decode('utf-8','replace'))
            self._trees[revision]=parse_tree_records(p.stdout);self._stats['tree_queries']+=1
        return self._trees[revision]

    def oid(self,revision,name):
        name=repository_path(name);row=self._tree(revision).get(name)
        if row is None:raise FileNotFoundError('Missing immutable tree path: '+name)
        if row['type']!='blob' or row['mode'] not in ('100644','100755'):
            raise ValueError('Expected a regular blob, not a symlink/submodule: '+name)
        return row['oid']

    def _start(self):
        if self._closed:raise RuntimeError('Reader already closed')
        if self._process is None:
            # A temporary stderr file cannot fill a pipe and block large reads.
            self._stderr=tempfile.TemporaryFile(mode='w+b',dir=self.temp)
            self._process=subprocess.Popen([self.git,'cat-file','--batch'],cwd=self.root,env=self.env,
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self._stderr,shell=False,
                creationflags=subprocess.CREATE_NO_WINDOW)
            self._stats['batch_processes']+=1

    def _request(self,oid,capture):
        immutable_oid(oid);self._start()
        self._process.stdin.write((oid+'\n').encode('ascii'));self._process.stdin.flush()
        # Send one small request then drain its full response; never enqueue a
        # large request list while stdout is blocked on a full Windows pipe.
        row=read_blob_frame(self._process.stdout,oid,capture=capture)
        self._stats['blob_requests']+=1;self._stats['raw_bytes_read']+=row['bytes']
        self._digests[oid]={k:row[k] for k in ('oid','bytes','sha256')}
        if capture and row['bytes']<=2*1024*1024:self._data[oid]=row['data']
        return row

    def signature(self,revision,name):
        oid=self.oid(revision,name)
        if oid in self._digests:
            self._stats['digest_cache_hits']+=1;return dict(self._digests[oid])
        return {k:v for k,v in self._request(oid,False).items() if k!='data'}

    def read(self,revision,name):
        oid=self.oid(revision,name)
        if oid in self._data:
            self._stats['byte_cache_hits']+=1;return self._data[oid]
        return self._request(oid,True)['data']

    def close(self,*,abort=False):
        if self._closed:return
        self._closed=True;error=None
        try:
            if self._process is not None:
                # Every successful request was drained. On parser failure,
                # close stdout too so an unfinished writer cannot block exit.
                for stream in (self._process.stdout,self._process.stdin):
                    try:stream.close()
                    except OSError as exc:error=RuntimeError('Read-only cat-file pipe close failed: '+str(exc))
                try:code=self._process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    self._process.kill();self._process.wait(timeout=10)
                    error=RuntimeError('Read-only cat-file failed to exit')
                else:
                    if code:
                        self._stderr.seek(0)
                        error=RuntimeError('Read-only cat-file failed: '+self._stderr.read().decode('utf-8','replace'))
        finally:
            if self._stderr is not None:self._stderr.close()
        if error is not None and not abort:raise error


def parser_self_test():
    """Pure byte fixtures and AST parsing only: no subprocess/Git invocation."""
    import ast
    import io
    tests=[]
    def record(name,condition):
        if not condition:raise AssertionError(name)
        tests.append(name)
    def rejected(name,operation):
        try:operation()
        except (BatchProtocolError,ValueError):tests.append(name)
        else:raise AssertionError('Expected rejection: '+name)
    def fixture(raw):
        oid=hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()
        return oid,oid.encode()+b' blob '+str(len(raw)).encode()+b'\n'+raw+b'\n'
    binary=b'first\n\0\xff last\r\n';oid,frame=fixture(binary)
    row=read_blob_frame(io.BytesIO(frame),oid,capture=True)
    record('binary_payload_with_nul_and_newlines',row['data']==binary and row['bytes']==len(binary) and row['sha256']==hashlib.sha256(binary).hexdigest())
    empty_oid,empty_frame=fixture(b'')
    record('empty_blob',read_blob_frame(io.BytesIO(empty_frame),empty_oid,capture=True)['data']==b'')
    stream=io.BytesIO(frame+empty_frame)
    read_blob_frame(stream,oid)
    record('consecutive_response_framing',read_blob_frame(stream,empty_oid)['bytes']==0 and stream.read()==b'')
    class ShortRead(io.BytesIO):
        def read(self,size=-1):return super().read(min(size,3) if size>=0 else 3)
    record('short_payload_reads',read_blob_frame(ShortRead(frame),oid,capture=True)['data']==binary)
    record('digest_only_does_not_keep_payload',read_blob_frame(io.BytesIO(frame),oid)['data'] is None)
    rejected('wrong_expected_oid',lambda:read_blob_frame(io.BytesIO(frame),'0'*40))
    rejected('non_blob_object',lambda:read_blob_frame(io.BytesIO(frame.replace(b' blob ',b' tree ',1)),oid))
    rejected('missing_object',lambda:read_blob_frame(io.BytesIO(oid.encode()+b' missing\n'),oid))
    rejected('missing_header_terminator',lambda:read_blob_frame(io.BytesIO(oid.encode()+b' blob 5'),oid))
    rejected('negative_size',lambda:read_blob_frame(io.BytesIO(oid.encode()+b' blob -1\n'),oid))
    rejected('truncated_payload',lambda:read_blob_frame(io.BytesIO(frame[:-4]),oid))
    rejected('missing_payload_terminator',lambda:read_blob_frame(io.BytesIO(frame[:-1]),oid))
    rejected('corrupted_payload_oid_mismatch',lambda:read_blob_frame(io.BytesIO(frame.replace(b'first',b'other')),oid))
    rejected('declared_size_limit',lambda:read_blob_frame(io.BytesIO(frame),oid,max_bytes=1))
    unicode_name='folder with spaces/한글 파일.bin'
    tree=b'100644 blob '+oid.encode()+b'\t'+unicode_name.encode()+b'\0'
    record('tree_space_unicode_path',parse_tree_records(tree)[unicode_name]['oid']==oid)
    rejected('tree_missing_nul',lambda:parse_tree_records(tree[:-1]))
    rejected('tree_duplicate_path',lambda:parse_tree_records(tree+tree))
    rejected('tree_traversal_path',lambda:parse_tree_records(b'100644 blob '+oid.encode()+b'\t../escape\0'))
    for expression in ('HEAD',':index','a'*39,'a'*40+':file','a'*40+'^{tree}'):
        rejected('mutable_or_expression_ref_'+expression,lambda expression=expression:immutable_oid(expression))
    for filename in ('save_checkpoint.py','save_review.py'):
        path=Path(__file__).resolve().parent/filename
        ast.parse(path.read_text(encoding='utf-8-sig'),filename=str(path))
        record('syntax_only_'+filename,True)
    return dict(passed=True,test_count=len(tests),tests=tests,
        subprocesses_started=0,git_commands_executed=0,repository_mutations=0,
        helper_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        scope='Synthetic binary protocol/parser checks and AST syntax only; live Git and commit workflow were not executed.')


if __name__=='__main__':
    import argparse
    import json
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test',action='store_true',required=True)
    parser.parse_args()
    print(json.dumps(parser_self_test(),indent=2,ensure_ascii=False),flush=True)
