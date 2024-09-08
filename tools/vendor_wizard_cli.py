import os
import json
import requests
from packaging import version
import argparse
import sys
from colorama import init, Fore, Style
import subprocess

# Vendor Wizard CLI Tool

# Initialize colorama
init(autoreset=True)

# Change to parent directory
os.chdir('..')

# Change to vendordeps directory
os.chdir('vendordeps')

def check_internet():
    try:
        subprocess.check_output(["ping", "-c", "1", "8.8.8.8"], stderr=subprocess.STDOUT)
        return True
    except subprocess.CalledProcessError:
        return False

def check_vendordep_update(local_json_path, online_json_url):
    try:
        with open(local_json_path, 'r') as file:
            local_data = json.load(file)
        response = requests.get(online_json_url)
        if response.status_code == 200:
            online_data = response.json()
        else:
            return None, f"Failed to fetch online JSON. Status code: {response.status_code}"
        local_version = version.parse(local_data['version'])
        online_version = version.parse(online_data['version'])
        if online_version > local_version:
            return (local_version, online_version), None
        else:
            return None, None
    except Exception as e:
        return None, str(e)

def update_vendordep(local_json_path, online_json_url):
    try:
        response = requests.get(online_json_url)
        if response.status_code == 200:
            with open(local_json_path, 'w') as file:
                file.write(response.text)
            print(f"  {Fore.GREEN}Updated {os.path.basename(local_json_path)} successfully!{Style.RESET_ALL}")
            return True
        else:
            print(f"  {Fore.RED}Failed to fetch online JSON. Status code: {response.status_code}{Style.RESET_ALL}")
            return False
    except Exception as e:
        print(f"  {Fore.RED}Error: {str(e)}{Style.RESET_ALL}")
        return False

def list_vendordeps(vendordeps_folder, update=False, gradle=False):
    if not check_internet():
        if not gradle:
            print(f"\n{Fore.YELLOW}⚠ No internet connection. Showing current vendordep versions only.{Style.RESET_ALL}")
        total_count = 0
        for file_name in os.listdir(vendordeps_folder):
            if file_name == 'WPILibNewCommands.json':
                continue  # Skip this specific file as it cannot be updated
            if file_name.endswith('.json'):
                local_json_path = os.path.join(vendordeps_folder, file_name)
                with open(local_json_path, 'r') as file:
                    local_data = json.load(file)
                total_count += 1
                if not gradle:
                    print(f"\n{Fore.CYAN}{Style.BRIGHT}{local_data['name']}:{Style.RESET_ALL}")
                    print(f"  Current: {Fore.YELLOW}{local_data['version']}{Style.RESET_ALL}")
                    print(f"  Status: {Fore.YELLOW}Unknown (No internet){Style.RESET_ALL}")
        if not gradle:
            print(f"\n{Style.BRIGHT}Summary: {Fore.YELLOW}Loaded {total_count} vendordeps. No internet connection.{Style.RESET_ALL}")
        return

    outdated_count = 0
    total_count = 0
    updated_count = 0
    for file_name in os.listdir(vendordeps_folder):
        if file_name == 'WPILibNewCommands.json':
            continue  # Skip this specific file as it cannot be updated
        if file_name.endswith('.json'):
            local_json_path = os.path.join(vendordeps_folder, file_name)
            with open(local_json_path, 'r') as file:
                local_data = json.load(file)
            online_json_url = local_data['jsonUrl']
            result, error = check_vendordep_update(local_json_path, online_json_url)

            total_count += 1
            if not gradle:
                print(f"\n{Fore.CYAN}{Style.BRIGHT}{local_data['name']}:{Style.RESET_ALL}")
            if error:
                if not gradle:
                    print(f"  {Fore.RED}Error: {error}{Style.RESET_ALL}")
            elif result:
                local_version, online_version = result
                if not gradle:
                    print(f"  Current: {Fore.RED}{local_version}{Style.RESET_ALL} | Latest: {Fore.GREEN}{online_version}{Style.RESET_ALL}")
                    print(f"  Status: {Fore.RED}Outdated{Style.RESET_ALL}")
                outdated_count += 1
                if update:
                    if update_vendordep(local_json_path, online_json_url):
                        updated_count += 1
            else:
                if not gradle:
                    print(f"  Current: {Fore.GREEN}{local_data['version']}{Style.RESET_ALL}")
                    print(f"  Status: {Fore.GREEN}Up to date{Style.RESET_ALL}")

    if not gradle:
        print(f"\n{Style.BRIGHT}Summary: {Fore.RED if outdated_count > 0 else Fore.GREEN}{outdated_count} out of {total_count} vendordeps are outdated.{Style.RESET_ALL}")
        if update:
            print(f"{Style.BRIGHT}Updated: {Fore.GREEN}{updated_count} vendordeps were successfully updated.{Style.RESET_ALL}")

def main():
    parser = argparse.ArgumentParser(description="Manage and update vendordeps for your FRC project.")
    parser.add_argument("--vendordeps", default=os.getcwd(), help="Path to vendordeps folder (default: current directory)")
    parser.add_argument("command", nargs="?", choices=["list", "update", "help"], help="Command to execute")
    parser.add_argument("-g", "--gradle", action="store_true", help="Run update without listing (for Gradle)")

    args = parser.parse_args()

    if args.command is None:
        print(f"{Fore.RED}Error: No command specified.{Style.RESET_ALL}")
        print(f"\n{Fore.YELLOW}Available commands:{Style.RESET_ALL}")
        print(f"  {Fore.CYAN}list{Style.RESET_ALL}   - Display all vendordeps and their status")
        print(f"  {Fore.CYAN}update{Style.RESET_ALL} - Update all outdated vendordeps")
        sys.exit(1)

    # Ensure the vendordeps directory exists
    if not os.path.isdir(args.vendordeps):
        print(f"{Fore.RED}Error: The specified vendordeps directory does not exist: {args.vendordeps}{Style.RESET_ALL}")
        sys.exit(1)

    if args.command == "list":
        list_vendordeps(args.vendordeps)
    elif args.command == "update":
        list_vendordeps(args.vendordeps, update=True, gradle=args.gradle)

if __name__ == "__main__":
    main()
