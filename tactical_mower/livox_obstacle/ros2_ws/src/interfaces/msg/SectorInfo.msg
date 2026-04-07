# Sector information for obstacle avoidance
int32 sector_id              # Sector identifier (0 to num_sectors-1)
float32 density              # 0.0-1.0, obstacle density in this sector
float32 min_distance         # [m] Minimum distance to obstacle in sector (inf if no obstacle)
string classification        # FREE, SPARSE, or BLOCKED
string sector_type           # STOP or SLOW - indicates required action type
