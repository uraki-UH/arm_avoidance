uint8 VALUE_FLOAT=0
uint8 VALUE_INT=1
uint8 VALUE_BOOL=2
uint8 VALUE_STRING=3
uint8 VALUE_FLOAT_ARRAY=4

std_msgs/Header header
string profile_name
string schema_id
uint32 schema_revision
string scope_type
string source_topic

# Metric definitions. All definition arrays have the same length.
string[] metric_ids
string[] labels
string[] units
uint8[] value_types
string[] metric_scope_types
string[] group_names
string[] descriptions
bool[] visible
bool[] enabled
float32[] default_weights

# Flattened samples. sample_metric_offsets length is sample_scope_ids.length + 1.
string[] sample_scope_ids
uint32[] sample_metric_offsets
string[] sample_metric_ids
bool[] sample_metric_valid
float64[] sample_metric_scalar_values
int64[] sample_int_values
bool[] sample_bool_values
string[] sample_string_values

# Array boundaries for each sample_metric_ids slot.
# sample_metric_array_offsets length is sample_metric_ids.length + 1.
# Slot i uses sample_metric_array_values[offsets[i]:offsets[i + 1]].
uint32[] sample_metric_array_offsets
float64[] sample_metric_array_values
