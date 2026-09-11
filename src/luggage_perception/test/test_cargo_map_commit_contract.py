from luggage_perception.cargo_volume_mapper import CargoVolumeMapper


def _mapper():
    return CargoVolumeMapper(
        inner_size=[1.0, 1.0, 1.0],
        center_base=[0.0, 0.0, 0.5],
        yaw=0.0,
        resolution=0.1,
    )


def test_add_duplicate_remove_reset_revision_ledger_and_surface_contract():
    mapper = _mapper()
    center = [0.0, 0.0, 0.15]
    size = [0.2, 0.2, 0.2]

    empty = mapper.stats()
    assert empty["occupancy_ratio"] == 0.0
    assert empty["occupied_volume"] == 0.0
    initial_revision = empty["map_revision"]

    assert mapper.mark_placed_box(center, size, yaw=0.0)
    marked = mapper.stats()
    assert marked["map_revision"] == initial_revision + 1
    assert marked["committed_box_count"] == 1
    assert marked["occupied_volume"] > 0.0
    assert max(max(row) for row in mapper.surface_map_2d()["height"]) >= size[2]

    assert not mapper.mark_placed_box(center, size, yaw=0.0)
    assert mapper.stats()["map_revision"] == marked["map_revision"]
    assert len(mapper.commit_ledger()) == 1

    ledger = mapper.commit_ledger()
    ledger[0]["center"][0] = 99.0
    assert mapper.commit_ledger()[0]["center"][0] == center[0]

    assert not mapper.unmark_placed_box([0.4, 0.4, 0.15], size, yaw=0.0)
    assert mapper.stats()["map_revision"] == marked["map_revision"]

    assert mapper.unmark_placed_box(center, size, yaw=0.0)
    removed = mapper.stats()
    assert removed["map_revision"] == marked["map_revision"] + 1
    assert removed["committed_box_count"] == 0
    assert removed["occupied_volume"] == 0.0
    assert max(max(row) for row in mapper.surface_map_2d()["height"]) == 0.0

    mapper.reset()
    assert mapper.stats()["map_revision"] == removed["map_revision"] + 1
