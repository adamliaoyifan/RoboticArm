"""Incremental canonical roots must match full evidence, across leaf seams."""
import hashlib
import numpy as np
import pytest

from luggage_perception.cargo_fact_digest import CanonicalFactDigest,LEAF_BYTES
from luggage_perception.cargo_voxel_delta import EVIDENCE_DTYPE


def independent_root(facts,active,members):
    payload=b'cargo-map-block-v2\0'+(len(facts)//EVIDENCE_DTYPE.itemsize).to_bytes(8,'little')
    payload+=b''.join(hashlib.sha256(facts[start:start+64*EVIDENCE_DTYPE.itemsize]).digest()
        for start in range(0,len(facts),64*EVIDENCE_DTYPE.itemsize))
    payload+=active
    for identifier in sorted(members):
        encoded=identifier.encode('utf-8')
        payload+=len(encoded).to_bytes(4,'little')+encoded
        payload+=len(members[identifier]).to_bytes(4,'little')+members[identifier]
    return hashlib.sha256(payload).hexdigest()


@pytest.mark.parametrize('seed',range(20))
def test_twenty_seeds_five_hundred_leaf_updates_match_full_root(seed):
    rng=np.random.default_rng(seed)
    records=np.zeros(131,dtype=EVIDENCE_DTYPE)
    facts=bytearray(records.tobytes())
    owner=CanonicalFactDigest.build(facts)
    active=bytes([1])*len(records)
    for transaction in range(500):
        offsets=np.unique(np.r_[rng.integers(0,len(records),3),[0,63,64,127,128,130][transaction%6]])
        field=EVIDENCE_DTYPE.names[transaction%len(EVIDENCE_DTYPE.names)]
        records[field][offsets]=rng.integers(0,3,len(offsets))
        if field in ('logodds','shadow') and transaction%7==0:
            records[field][offsets[0]]=np.nan
        before=owner.leaves
        facts[:]=records.tobytes()
        owner=owner.updated(facts,offsets)
        members={'box-b':b'\x01\x00','box-a':b'\x03\x00'} if transaction%3 else {}
        assert owner.block_digest(active,members)==independent_root(bytes(facts),active,members)
        untouched=set(range(len(before)//32))-set(offsets//64)
        for leaf in untouched:
            assert before[leaf*32:(leaf+1)*32]==owner.leaves[leaf*32:(leaf+1)*32]
    assert owner.byte_size()>=len(owner.leaves)


def test_active_memberships_and_final_partial_leaf_are_bound():
    facts=bytearray(LEAF_BYTES+EVIDENCE_DTYPE.itemsize)
    owner=CanonicalFactDigest.build(facts)
    first=owner.block_digest(bytes([1])*65,{})
    assert first!=owner.block_digest(bytes(65),{})
    assert first!=owner.block_digest(bytes([1])*65,{'real-box':b'\x40\x00'})
    facts[-1]=1
    changed=owner.updated(facts,np.asarray([64]))
    assert owner.leaves[:32]==changed.leaves[:32]
    assert first!=changed.block_digest(bytes([1])*65,{})


def test_dirty_leaf_bitmap_accepts_unsorted_and_duplicate_record_offsets():
    facts = bytearray(131*EVIDENCE_DTYPE.itemsize)
    owner = CanonicalFactDigest.build(facts)
    offsets = np.asarray([130, 64, 0, 64, 127, 130])
    for offset in offsets:
        facts[int(offset)*EVIDENCE_DTYPE.itemsize] = 1
    assert owner.updated(facts, offsets) == CanonicalFactDigest.build(facts)
