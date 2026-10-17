# Valid alternative: the reference migration, committed (agents may commit; the checks compare with the build commit).
O="$(dirname "$0")/../solution/files"
cp -a "$O/." .
git -c user.name=agent -c user.email=agent@localhost add -A
git -c user.name=agent -c user.email=agent@localhost commit -qm "Migrate Trail Walker rewards to ProxyArray accessors"
