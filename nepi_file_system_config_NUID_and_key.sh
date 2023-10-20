#!/bin/bash

# This bash scrict updates a NEPI device's NUID and Public Key
# You must reboot your device after updating for changes to take effect

#####################################################
### Edit NUID Options Below
#####################################################
###--NUID Updating -------------------------------------------------------
### This should be a unique number suggest using the following
### link to create a random number
### https://numbergenerator.org/random-10-digit-number-generator 
#NUID=4999999999
### or randomize it -- this does a random 10-digit number
NUID=`echo $((RANDOM%(9999)))$((RANDOM%(9999)))$((RANDOM%(99)))`
#####################################################

### Change the NUID number below to your desired NUID
folder=/opt/nepi/nepi_link/nepi-bot/devinfo
echo $NUID > $folder/devnuid.txt
rm $folder/devsshkeys.txt $folder/*pub
ssh-keygen -f $folder/id_rsa -N ""
mv $folder/id_rsa $folder/devsshkeys.txt
mv $folder/id_rsa.pub $folder/id_rsa_${NUID}.pub

