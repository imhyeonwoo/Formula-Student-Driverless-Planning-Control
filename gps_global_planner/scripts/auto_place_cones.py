#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
센터라인(test1.csv) → 좌‧우 콘 CSV(placed_cones_temp.csv) 자동 생성
  • lane_width  : 좌~우 전체 폭 [m]  (기본 5)
  • cone_spacing: 같은 사이드 콘 간격 [m] (기본 8)
"""

import math, os, argparse
import numpy as np, pandas as pd

R_EARTH = 6_378_137.0

def latlon_to_local(lat, lon, ref_lat, ref_lon):
    lat_r, lon_r   = math.radians(lat) , math.radians(lon)
    ref_lat_r, ref_lon_r = math.radians(ref_lat), math.radians(ref_lon)
    x = (lon_r - ref_lon_r) * math.cos(ref_lat_r) * R_EARTH
    y = (lat_r - ref_lat_r) * R_EARTH
    return x, y

def local_to_latlon(x, y, ref_lat, ref_lon):
    ref_lat_r, ref_lon_r = math.radians(ref_lat), math.radians(ref_lon)
    lat_r = y / R_EARTH + ref_lat_r
    lon_r = x / (R_EARTH * math.cos(ref_lat_r)) + ref_lon_r
    return math.degrees(lat_r), math.degrees(lon_r)

def main():
    script_dir   = os.path.abspath(os.path.dirname(__file__))
    data_dir     = os.path.join(script_dir, "..", "data")
    default_in   = os.path.join(data_dir, "administrator_250721.csv")
    default_out  = os.path.join(data_dir, "administrator_250721_cones.csv")

    parser = argparse.ArgumentParser()
    parser.add_argument("--input_csv",  default=default_in)
    parser.add_argument("--output_csv", default=default_out)
    parser.add_argument("--lane_width", type=float, default=5.0)
    parser.add_argument("--cone_spacing", type=float, default=2.0)
    args = parser.parse_args()

    df = pd.read_csv(args.input_csv)
    lat_col = next(c for c in df.columns if "lat" in c.lower())
    lon_col = next(c for c in df.columns if "lon" in c.lower())
    lats, lons = df[lat_col].to_numpy(), df[lon_col].to_numpy()

    ref_lat, ref_lon = lats[0], lons[0]
    xy = np.array([latlon_to_local(lat, lon, ref_lat, ref_lon)
                   for lat, lon in zip(lats, lons)])

    seg_len = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    cum_len = np.insert(np.cumsum(seg_len), 0, 0.0)

    half_w, spacing = args.lane_width / 2.0, args.cone_spacing
    next_d = 0.0
    cones_l, cones_r = [], []

    for i in range(len(xy) - 1):
        p0, p1 = xy[i], xy[i+1]
        vec, dist = p1 - p0, seg_len[i]
        while next_d <= cum_len[i]:
            t = (next_d - cum_len[i]) / dist if dist > 0 else 0
            pos = p0 + t * vec
            dir_v = vec / dist if dist > 0 else np.array([1, 0])
            perp  = np.array([-dir_v[1], dir_v[0]])
            cones_l.append(pos + half_w * perp)
            cones_r.append(pos - half_w * perp)
            next_d += spacing

    cones_l.pop(0)
    cones_r.pop(0)

    cones_xy = np.vstack([cones_l, cones_r])
    cone_latlon = [local_to_latlon(x, y, ref_lat, ref_lon) for x, y in cones_xy]
    pd.DataFrame(cone_latlon, columns=["lat", "lon"]).to_csv(args.output_csv, index=False)
    print(f"✅  {len(cone_latlon)} cones saved → {os.path.abspath(args.output_csv)}")

if __name__ == "__main__":
    main()