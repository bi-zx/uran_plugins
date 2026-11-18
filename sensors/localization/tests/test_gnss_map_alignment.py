from uran_localization_plugin import GeoReferenceConfig, GnssMapAlignment, MapProjector


def test_revisiting_a_location_replaces_its_early_gnss_outlier():
    base = MapProjector(GeoReferenceConfig(enabled=True, reference_lat=39, reference_lon=116))
    fit = GnssMapAlignment("route", (116,39), None, {})
    def sample(x, offset, second):
        lat,lon = base.map_xy_to_latlon(x+offset,0)
        return fit.update({"lat":lat,"lon":lon,"horizontal_fix_valid":True}, {"x":x,"y":0}, second*10**9)
    for second, (x, error) in enumerate([(0,30),(2,0),(4,0)], 1):
        assert not sample(x,error,second)
    assert sample(0,0,5)
    assert fit.inliers == 3 and len(fit.sample_positions) == 3
    assert abs(fit.projector.latlon_to_map_xy(*base.map_xy_to_latlon(4,0))[0]-4) < 1e-5
