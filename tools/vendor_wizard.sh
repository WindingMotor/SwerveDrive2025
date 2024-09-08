#!/bin/bash

# Vendor Wizard CLI Tool in Bash

# Change to parent directory
cd ..

# Change to vendordeps directory
cd vendordeps

# Check for internet connection
check_internet() {
    if ping -c 1 8.8.8.8 &> /dev/null; then
        return 0
    else
        return 1
    fi
}

# Check for vendordep update
check_vendordep_update() {
    local_json_path=$1
    online_json_url=$2

    local_version=$(jq -r '.version' "$local_json_path")
    online_data=$(curl -s "$online_json_url")
    if [ $? -ne 0 ]; then
        echo "Failed to fetch online JSON"
        return 1
    fi

    online_version=$(echo "$online_data" | jq -r '.version')
    if [ "$(printf '%s\n' "$online_version" "$local_version" | sort -V | head -n1)" != "$online_version" ]; then
        echo "$local_version $online_version"
        return 0
    else
        return 2
    fi
}

# Update vendordep
update_vendordep() {
    local_json_path=$1
    online_json_url=$2

    if curl -s "$online_json_url" -o "$local_json_path"; then
        echo "Updated $(basename "$local_json_path") successfully!"
        return 0
    else
        echo "Failed to fetch online JSON"
        return 1
    fi
}

# List vendordeps
list_vendordeps() {
    local vendordeps_folder=$1
    local update=$2
    local gradle=$3

    if ! check_internet; then
        if [ "$gradle" = true ]; then
            echo "No Internet, cannot check vendordeps"
        else
            echo "⚠ No internet connection. Showing current vendordep versions only."

            local total_count=0
            for file_name in "$vendordeps_folder"/*.json; do
                if [ "$(basename "$file_name")" == "WPILibNewCommands.json" ]; then
                    continue
                fi

                local_data=$(jq '.' "$file_name")
                local_version=$(echo "$local_data" | jq -r '.version')
                local_name=$(echo "$local_data" | jq -r '.name')
                total_count=$((total_count + 1))

                echo "$local_name:"
                echo "  Current: $local_version"
                echo "  Status: Unknown (No internet)"
            done

            echo "Summary: Loaded $total_count vendordeps. No internet connection."
        fi
        return
    fi

    local outdated_count=0
    local total_count=0
    local updated_count=0
    for file_name in "$vendordeps_folder"/*.json; do
        if [ "$(basename "$file_name")" == "WPILibNewCommands.json" ]; then
            continue
        fi

        local_data=$(jq '.' "$file_name")
        local_version=$(echo "$local_data" | jq -r '.version')
        local_name=$(echo "$local_data" | jq -r '.name')
        online_json_url=$(echo "$local_data" | jq -r '.jsonUrl')
        total_count=$((total_count + 1))

        result=$(check_vendordep_update "$file_name" "$online_json_url")
        status=$?

        if [ "$gradle" = false ]; then
            echo "$local_name:"
        fi

        if [ $status -eq 1 ]; then
            if [ "$gradle" = false ]; then
                echo "  Error: Failed to fetch online JSON"
            fi
        elif [ $status -eq 0 ]; then
            local_version=$(echo "$result" | cut -d ' ' -f1)
            online_version=$(echo "$result" | cut -d ' ' -f2)

            if [ "$gradle" = false ]; then
                echo "  Current: $local_version | Latest: $online_version"
                echo "  Status: Outdated"
            fi

            outdated_count=$((outdated_count + 1))

            if [ "$update" = true ]; then
                if update_vendordep "$file_name" "$online_json_url"; then
                    updated_count=$((updated_count + 1))
                fi
            fi
        else
            if [ "$gradle" = false ]; then
                echo "  Current: $local_version"
                echo "  Status: Up to date"
            fi
        fi
    done

    if [ "$gradle" = true ]; then
        echo "Updated: $updated_count vendordeps successfully."
    else
        echo "Summary: $outdated_count out of $total_count vendordeps are outdated."
        if [ "$update" = true ]; then
            echo "Updated: $updated_count vendordeps were successfully updated."
        fi
    fi
}

# Main function
main() {
    local vendordeps_folder="."
    local command=""
    local gradle=false

    while [[ $# -gt 0 ]]; do
        case $1 in
            --vendordeps)
                vendordeps_folder="$2"
                shift
                shift
                ;;
            list|update|help)
                command="$1"
                shift
                ;;
            -g|--gradle)
                gradle=true
                shift
                ;;
            *)
                shift
                ;;
        esac
    done

    if [ -z "$command" ]; then
        echo "Error: No command specified."
        echo "Available commands:"
        echo "  list   - Display all vendordeps and their status"
        echo "  update - Update all outdated vendordeps"
        exit 1
    fi

    if [ ! -d "$vendordeps_folder" ]; then
        echo "Error: The specified vendordeps directory does not exist: $vendordeps_folder"
        exit 1
    fi

    if [ "$command" == "list" ]; then
        list_vendordeps "$vendordeps_folder" false "$gradle"
    elif [ "$command" == "update" ]; then
        list_vendordeps "$vendordeps_folder" true "$gradle"
    fi
}

main "$@"
