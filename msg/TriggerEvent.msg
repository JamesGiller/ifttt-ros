Header header
# Requests to the IFTTT Webhooks service can include a JSON body of
# { "value1": "...", "value2": "...", "value3": "..." }
# whose value properties will be passed on to the action in a recipe.
# Only the first three elements of the array will be used to populate the JSON request body.
string[] values